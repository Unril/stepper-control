#include "stdafx.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../include/sc/GCodeParser.h"

using namespace StepperControl;
using namespace testing;

namespace {

struct AxTr {
    static const int size = 3;
    static const char *names() { return "AXC"; }
};

using Af = TAf<AxTr::size>;

struct GCodeParserCallbacksMock {
    MOCK_METHOD1(feedrateOverride, void(float));
    MOCK_METHOD2(linearMove, void(Af const &, float));
    MOCK_METHOD1(g0RapidMove, void(Af const &));
    MOCK_METHOD2(g1LinearMove, void(Af const &, float));
    MOCK_METHOD1(g4Wait, void(float));
    MOCK_METHOD0(g28RunHomingCycle, void());
    MOCK_METHOD1(g90g91DistanceMode, void(DistanceMode));
    MOCK_METHOD1(m100MaxVelocityOverride, void(Af const &));
    MOCK_METHOD1(m101MaxAccelerationOverride, void(Af const &));
    MOCK_METHOD1(m102StepsPerUnitLengthOverride, void(Af const &));
    MOCK_METHOD1(m103HomingVelocityOverride, void(Af const &));
    MOCK_METHOD1(m105MinPositionOverride, void(Af const &));
    MOCK_METHOD1(m106MaxPositionOverride, void(Af const &));
    MOCK_METHOD0(m110PrintAxesConfiguration, void());
    MOCK_METHOD0(start, void());
    MOCK_METHOD0(stop, void());
    MOCK_CONST_METHOD0(isRunning, bool());
    MOCK_CONST_METHOD0(printCurrentPosition, void());
    MOCK_CONST_METHOD0(m104PrintInfo, void());
    MOCK_METHOD0(clearCommandsBuffer, void());

    void error(const char *reason) {
        std::stringstream ss;
        ss << "Error " << reason << std::endl;
        throw std::logic_error(ss.str());
    }
};

struct GCodeParser_Should : Test {
    GCodeParser_Should() {}

    GCodeParserCallbacksMock cb_;
    GCodeParser<GCodeParserCallbacksMock, AxTr> parser_{&cb_};

    void parse(std::string const &line) { parser_.parseLine(line.c_str()); }
};

TEST_F(GCodeParser_Should, parse_empty_line) {
    parse("\n");
    parse("\t  \n");
}

TEST_F(GCodeParser_Should, not_parse_empty_string) {
    EXPECT_THROW(parse(""), std::logic_error);
    EXPECT_THROW(parse("\t  "), std::logic_error);
}

TEST_F(GCodeParser_Should, parse_linear_move) {
    EXPECT_CALL(cb_, linearMove(ElementsAre(1.f, 2.f, 3.f), Eq(inf())));
    parse("A1 X2 C3\n");

    EXPECT_CALL(cb_, linearMove(ElementsAre(10.f, 20.f, 30.f), Eq(inf())));
    parse("a10 x20 c30\n");
}

TEST_F(GCodeParser_Should, parse_linear_move_with_feedrate) {
    EXPECT_CALL(cb_, linearMove(ElementsAre(1.f, -2.f, 3.f), Eq(123.f)));
    parse("A1 X-2 C+3 F123\n");

    EXPECT_CALL(cb_, linearMove(ElementsAre(1.f, inf(), inf()), Eq(0.2f)));
    parse("a1 f.2\n");
}

TEST_F(GCodeParser_Should, parse_linear_move_with_floating_value) {
    EXPECT_CALL(cb_, linearMove(ElementsAre(1.2f, -0.3f, 1e+2f), Eq(inf())));
    parse("A1.2 X-.3 C1e+2\n");
}

TEST_F(GCodeParser_Should, set_unused_axes_to_inf_in_linear_move) {
    EXPECT_CALL(cb_, linearMove(ElementsAre(1.f, inf(), 2.f), Eq(inf())));
    parse("A1 C2\n");
}

TEST_F(GCodeParser_Should, parse_feedrate_override) {
    EXPECT_CALL(cb_, feedrateOverride(Eq(123.45f)));
    parse("F123.45\n");
}

TEST_F(GCodeParser_Should, parse_g0RapidMove) {
    EXPECT_CALL(cb_, g0RapidMove(ElementsAre(1.1f, 2.f, 3.f)));
    parse("G0 A1.1 X2 C3\n");

    EXPECT_CALL(cb_, g0RapidMove(ElementsAre(inf(), 2.f, inf())));
    parse("g0 x2\n");

    EXPECT_CALL(cb_, g0RapidMove(ElementsAre(inf(), inf(), inf())));
    parse("g0\n");
}

TEST_F(GCodeParser_Should, parse_g1LinearMove) {
    EXPECT_CALL(cb_, g1LinearMove(ElementsAre(-1.1f, 2.f, 3.f), Eq(4.1f)));
    parse("G1 A-1.1 X2 C3 F4.1\n");

    EXPECT_CALL(cb_, g1LinearMove(ElementsAre(inf(), 2.f, inf()), Eq(inf())));
    parse("G1 X2\n");

    EXPECT_CALL(cb_, g1LinearMove(ElementsAre(inf(), inf(), inf()), Eq(inf())));
    parse("G1\n");
}

TEST_F(GCodeParser_Should, parse_g4Wait) {
    EXPECT_CALL(cb_, g4Wait(Eq(1.2f)));
    parse("G4 P1.2\n");

    EXPECT_CALL(cb_, g4Wait(Eq(100.f)));
    parse("g4 p100\n");
}

TEST_F(GCodeParser_Should, parse_g28RunHomingCycle) {
    EXPECT_CALL(cb_, g28RunHomingCycle());
    parse("G28\n");
}

TEST_F(GCodeParser_Should, parse_g90g91DistanceMode) {
    EXPECT_CALL(cb_, g90g91DistanceMode(Eq(DistanceMode::Absolute)));
    parse("G90\n");

    EXPECT_CALL(cb_, g90g91DistanceMode(Eq(DistanceMode::Relative)));
    parse("G91\n");
}

TEST_F(GCodeParser_Should, parse_m100MaxVelocityOverride) {
    EXPECT_CALL(cb_, m100MaxVelocityOverride(ElementsAre(3.14f, 0.11f, 0.123f)));
    parse("M100 A3.14 X0.11 C.123\n");

    EXPECT_CALL(cb_, m100MaxVelocityOverride(ElementsAre(inf(), inf(), 0.1f)));
    parse("M100 C.1\n");

    EXPECT_CALL(cb_, m100MaxVelocityOverride(ElementsAre(inf(), inf(), inf())));
    parse("M100\n");
}

TEST_F(GCodeParser_Should, parse_m101MaxAccelerationOverride) {
    EXPECT_CALL(cb_, m101MaxAccelerationOverride(ElementsAre(3.14f, 0.123f, 0.1f)));
    parse("M101 A3.14 X0.123 C.1\n");

    EXPECT_CALL(cb_, m101MaxAccelerationOverride(ElementsAre(inf(), 0.123f, inf())));
    parse("M101 X0.123\n");

    EXPECT_CALL(cb_, m101MaxAccelerationOverride(ElementsAre(inf(), inf(), inf())));
    parse("M101\n");
}

TEST_F(GCodeParser_Should, parse_m102StepsPerUnitLengthOverride) {
    EXPECT_CALL(cb_, m102StepsPerUnitLengthOverride(ElementsAre(3.14f, 0.123f, 0.1f)));
    parse("M102 A3.14 X0.123 C.1\n");

    EXPECT_CALL(cb_, m102StepsPerUnitLengthOverride(ElementsAre(0.123f, inf(), inf())));
    parse("M102 A0.123\n");

    EXPECT_CALL(cb_, m102StepsPerUnitLengthOverride(ElementsAre(inf(), inf(), inf())));
    parse("M102\n");
}

TEST_F(GCodeParser_Should, parse_m103HomingVelocityOverride) {
    EXPECT_CALL(cb_, m103HomingVelocityOverride(ElementsAre(3.14f, 0.123f, 0.1f)));
    parse("M103 A3.14 X0.123 C.1\n");

    EXPECT_CALL(cb_, m103HomingVelocityOverride(ElementsAre(0.123f, inf(), inf())));
    parse("M103 A0.123\n");

    EXPECT_CALL(cb_, m103HomingVelocityOverride(ElementsAre(inf(), inf(), inf())));
    parse("M103\n");
}

TEST_F(GCodeParser_Should, parse_m105MinPositionOverride) {
    EXPECT_CALL(cb_, m105MinPositionOverride(ElementsAre(3.14f, 0.123f, 0.1f)));
    parse("M105 A3.14 X0.123 C.1\n");

    EXPECT_CALL(cb_, m105MinPositionOverride(ElementsAre(0.123f, inf(), inf())));
    parse("M105 A0.123\n");

    EXPECT_CALL(cb_, m105MinPositionOverride(ElementsAre(inf(), inf(), inf())));
    parse("M105\n");
}

TEST_F(GCodeParser_Should, parse_m106MaxPositionOverride) {
    EXPECT_CALL(cb_, m106MaxPositionOverride(ElementsAre(3.14f, 0.123f, 0.1f)));
    parse("M106 A3.14 X0.123 C.1\n");

    EXPECT_CALL(cb_, m106MaxPositionOverride(ElementsAre(0.123f, inf(), inf())));
    parse("M106 A0.123\n");

    EXPECT_CALL(cb_, m106MaxPositionOverride(ElementsAre(inf(), inf(), inf())));
    parse("M106\n");
}

TEST_F(GCodeParser_Should, parse_m110PrintAxesConfiguration) {
    EXPECT_CALL(cb_, m110PrintAxesConfiguration());
    parse("M110\n");
}

TEST_F(GCodeParser_Should, not_parse_without_line_break) {
    EXPECT_THROW(parse("G1"), std::logic_error);
}

TEST_F(GCodeParser_Should, not_parse_repeated_codes) {
    EXPECT_THROW(parse("G1 G1\n"), std::logic_error);
}

TEST_F(GCodeParser_Should, not_parse_g_code_without_number) {
    EXPECT_THROW(parse("G\n"), std::logic_error);
}

TEST_F(GCodeParser_Should, not_parse_g0_code_with_floating_or_negative_number) {
    EXPECT_THROW(parse("G1.3\n"), std::logic_error);
    EXPECT_THROW(parse("G-1\n"), std::logic_error);
}

TEST_F(GCodeParser_Should, not_parse_g0_code_with_axes_without_values) {
    EXPECT_THROW(parse("G0 A X2\n"), std::logic_error);
}

TEST_F(GCodeParser_Should, not_parse_feedrate_without_number) {
    EXPECT_THROW(parse("F\n"), std::logic_error);
}

TEST_F(GCodeParser_Should, not_parse_g4Wait_without_number) {
    EXPECT_THROW(parse("G4 P\n"), std::logic_error);
}

TEST_F(GCodeParser_Should, not_parse_numbers_or_words) {
    EXPECT_THROW(parse("t\n"), std::logic_error);
    EXPECT_THROW(parse("1\n"), std::logic_error);
    EXPECT_THROW(parse(".\n"), std::logic_error);
    EXPECT_THROW(parse("+\n"), std::logic_error);
    EXPECT_THROW(parse("-\n"), std::logic_error);
    EXPECT_THROW(parse("&&\n"), std::logic_error);
}

TEST_F(GCodeParser_Should, skip_tabs_and_spaces) {
    EXPECT_CALL(cb_, linearMove(ElementsAre(-1000.f, 123.f, inf()), Eq(2.34f)));
    parse("\t\tA\t-1000   X  123 \t F+2.34\t \n");
}

TEST_F(GCodeParser_Should, start) {
    EXPECT_CALL(cb_, start());
    parse("~\n");
}

TEST_F(GCodeParser_Should, not_start_if_no_line_end) { EXPECT_THROW(parse("~"), std::logic_error); }

TEST_F(GCodeParser_Should, stop) {
    EXPECT_CALL(cb_, stop());
    parse("!\n");
}

TEST_F(GCodeParser_Should, not_stop_if_no_line_end) { EXPECT_THROW(parse("!"), std::logic_error); }

TEST_F(GCodeParser_Should, clearCommandsBuffer) {
    EXPECT_CALL(cb_, clearCommandsBuffer());
    parse("^\n");
}

TEST_F(GCodeParser_Should, not_clearCommandsBuffer_if_no_line_end) {
    EXPECT_THROW(parse("^"), std::logic_error);
}

TEST_F(GCodeParser_Should, printCurrentPosition) {
    EXPECT_CALL(cb_, printCurrentPosition());
    parse("?\n");
}

TEST_F(GCodeParser_Should, not_printCurrentPosition_if_no_line_end) {
    EXPECT_THROW(parse("?"), std::logic_error);
}

TEST_F(GCodeParser_Should, printInfo) {
    EXPECT_CALL(cb_, m104PrintInfo());
    parse("m104\n");
}

TEST_F(GCodeParser_Should, not_printInfo_if_no_line_end) {
    EXPECT_THROW(parse("m104"), std::logic_error);
}

TEST_F(GCodeParser_Should, parse_axes_written_together) {
    EXPECT_CALL(cb_, linearMove(ElementsAre(10.f, 20.f, 30.f), Eq(inf())));
    parse("a10x20c30\n");
}

TEST_F(GCodeParser_Should, parse_zero_axes_written_together) {
    EXPECT_CALL(cb_, linearMove(ElementsAre(0.f, 0.f, 0.f), Eq(inf())));
    parse("a0x0c0\n");
}

TEST_F(GCodeParser_Should, parse_axes_in_differen_order) {
    EXPECT_CALL(cb_, linearMove(ElementsAre(10.f, 20.f, 30.f), Eq(inf())));
    parse("c30x20a10\n");
}

TEST_F(GCodeParser_Should, parse_a0x1) {
    EXPECT_CALL(cb_, linearMove(ElementsAre(0.f, 1.f, inf()), Eq(inf())));
    parse("a0x1\r\n");
}

TEST_F(GCodeParser_Should, parse_lines_with_axes_written_together) {
    Sequence s;
    EXPECT_CALL(cb_, linearMove(ElementsAre(0.f, 10.f, inf()), Eq(inf()))).Times(1).InSequence(s);
    EXPECT_CALL(cb_, linearMove(ElementsAre(1.f, 10.f, inf()), Eq(inf()))).Times(1).InSequence(s);
    EXPECT_CALL(cb_, linearMove(ElementsAre(0.f, 0.f, inf()), Eq(inf()))).Times(1).InSequence(s);

    parse("a0x10\n");
    parse("a1x10\n");
    parse("a0x0\n");
}

}
