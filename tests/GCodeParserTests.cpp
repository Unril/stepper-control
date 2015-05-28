#include "stdafx.h"

#include "../include/sc/GCodeParser.h"

using namespace StepperControl;
using namespace testing;

namespace {
const size_t AxesSize = 3;

struct GCodeParserCallbacksMock : GCodeParserCallbacks<AxesSize> {
    MOCK_METHOD1(feedrateOverride, void(float));
    MOCK_METHOD2(linearMove, void(Axes<float, AxesSize> const &, float));
    MOCK_METHOD1(g0RapidMove, void(Axes<float, AxesSize> const &));
    MOCK_METHOD2(g1LinearMove, void(Axes<float, AxesSize> const &, float));
    MOCK_METHOD1(g4Wait, void(float));
    MOCK_METHOD0(g28RunHomingCycle, void());
    MOCK_METHOD1(g90g91DistanceMode, void(DistanceMode));
    MOCK_METHOD1(m100MaxVelocityOverride, void(Axes<float, AxesSize> const &));
    MOCK_METHOD1(m101MaxAccelerationOverride, void(Axes<float, AxesSize> const &));
    MOCK_METHOD1(m102StepsPerUnitLengthOverride, void(Axes<float, AxesSize> const &));
};

struct GCodeParser_Should : Test {
    GCodeParser_Should() {}

    GCodeParserCallbacksMock cb_;
    GCodeParser<AxesSize> parser_{&cb_};

    void parse(std::string const &line) {
        if (!parser_.parseLine(line.c_str())) {
            throw std::logic_error("Error pos = " + std::to_string(parser_.errorPosition()));
        }
    }
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
    parse("A1 B2 C3\n");

    EXPECT_CALL(cb_, linearMove(ElementsAre(10.f, 20.f, 30.f), Eq(inf())));
    parse("a10 b20 c30\n");
}

TEST_F(GCodeParser_Should, parse_linear_move_with_feedrate) {
    EXPECT_CALL(cb_, linearMove(ElementsAre(1.f, -2.f, 3.f), Eq(123.f)));
    parse("A1 B-2 C+3 F123\n");

    EXPECT_CALL(cb_, linearMove(ElementsAre(1.f, inf(), inf()), Eq(0.2f)));
    parse("a1 f.2\n");
}

TEST_F(GCodeParser_Should, parse_linear_move_with_floating_value) {
    EXPECT_CALL(cb_, linearMove(ElementsAre(1.2f, -0.3f, 1e+2f), Eq(inf())));
    parse("A1.2 B-.3 C1e+2\n");
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
    parse("G0 A1.1 B2 C3\n");

    EXPECT_CALL(cb_, g0RapidMove(ElementsAre(inf(), 2.f, inf())));
    parse("g0 b2\n");

    EXPECT_CALL(cb_, g0RapidMove(ElementsAre(inf(), inf(), inf())));
    parse("g0\n");
}

TEST_F(GCodeParser_Should, parse_g1LinearMove) {
    EXPECT_CALL(cb_, g1LinearMove(ElementsAre(-1.1f, 2.f, 3.f), Eq(4.1f)));
    parse("G1 A-1.1 B2 C3 F4.1\n");

    EXPECT_CALL(cb_, g1LinearMove(ElementsAre(inf(), 2.f, inf()), Eq(inf())));
    parse("G1 B2\n");

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
    EXPECT_CALL(cb_, g90g91DistanceMode(Eq(Absolute)));
    parse("G90\n");

    EXPECT_CALL(cb_, g90g91DistanceMode(Eq(Relative)));
    parse("G91\n");
}

TEST_F(GCodeParser_Should, parse_m100MaxVelocityOverride) {
    EXPECT_CALL(cb_, m100MaxVelocityOverride(ElementsAre(3.14f, 0.11f, 0.123f)));
    parse("M100 A3.14 B0.11 C.123\n");

    EXPECT_CALL(cb_, m100MaxVelocityOverride(ElementsAre(inf(), inf(), 0.1f)));
    parse("M100 C.1\n");

    EXPECT_CALL(cb_, m100MaxVelocityOverride(ElementsAre(inf(), inf(), inf())));
    parse("M100\n");
}

TEST_F(GCodeParser_Should, parse_m101MaxAccelerationOverride) {
    EXPECT_CALL(cb_, m101MaxAccelerationOverride(ElementsAre(3.14f, 0.123f, 0.1f)));
    parse("M101 A3.14 B0.123 C.1\n");

    EXPECT_CALL(cb_, m101MaxAccelerationOverride(ElementsAre(inf(), 0.123f, inf())));
    parse("M101 B0.123\n");

    EXPECT_CALL(cb_, m101MaxAccelerationOverride(ElementsAre(inf(), inf(), inf())));
    parse("M101\n");
}

TEST_F(GCodeParser_Should, parse_m102StepsPerUnitLengthOverride) {
    EXPECT_CALL(cb_, m102StepsPerUnitLengthOverride(ElementsAre(3.14f, 0.123f, 0.1f)));
    parse("M102 A3.14 B0.123 C.1\n");

    EXPECT_CALL(cb_, m102StepsPerUnitLengthOverride(ElementsAre(0.123f, inf(), inf())));
    parse("M102 A0.123\n");

    EXPECT_CALL(cb_, m102StepsPerUnitLengthOverride(ElementsAre(inf(), inf(), inf())));
    parse("M102\n");
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
    EXPECT_THROW(parse("G0 A B2\n"), std::logic_error);
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
    parse("\t\tA\t-1000   B  123 \t F+2.34\t \n");
}
}
