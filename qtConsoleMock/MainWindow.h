#pragma once

#include "../include/sc/Interfaces.h"
#include "../include/sc/GCodeParser.h"
#include "../include/sc/GCodeInterpreter.h"
#include "../include/sc/SegmentsExecutor.h"

#include <QMainWindow>
#include <QTimer>
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <functional>
#include <memory>

#include "GeneratedFiles/ui_MainWindow.h"

using namespace StepperControl;

struct Motor {
    template <size_t i, size_t edge>
    static void writeStep(UIntConst<i>, UIntConst<edge>) {}
    template <size_t i, size_t dir>
    static void writeDirection(UIntConst<i>, UIntConst<dir>) {}
    static bool checkEndSwitchHit(size_t i) { return true; }
    static void begin() {}
    static void end() {}
};

struct Ticker {
    explicit Ticker(QTimer *parent) : timer_(parent) {
        QObject::connect(timer_, &QTimer::timeout, [this]() { ticker_(); });
    }

    template <typename T>
    void attach_us(T *tptr, void (T::*mptr)(void), int time_us) {
        ticker_ = [=]() { (tptr->*mptr)(); };
        timer_->start();
    }
    void detach() {
        timer_->stop();
        ticker_ = []() {};
    }

    QTimer *timer_;
    std::function<void()> ticker_ = []() {};
};

struct SerialPrinter : Printer {
    explicit SerialPrinter(QSerialPort *port) : port_(port) {}

    void print(int n) override { write(n); }
    void print(size_t n) override { write(n); }
    void print(float n) override { write(n); }
    void print(double n) override { write(n); }
    void print(const char *str) override { write(str); }

    template <typename T>
    void write(T n) {
        Printer::print(n);
        QByteArray data;
        QTextStream(&data) << n;
        port_->write(data);
    }

    QSerialPort *port_;
};

const size_t axesSize = 5;

class MainWindow : public QMainWindow {
    Q_OBJECT

  public:
    using This = MainWindow;

    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

  public slots:
    void on_pbRefresh_clicked();
    void on_pbConnect_clicked();
    void on_pbDisconnect_clicked();

    void readyRead();
    void statusTimerTimeout();
    void executionStarted();
    void executionStopped();

  private:
    std::unique_ptr<SerialPrinter> printer_;
    std::unique_ptr<Ticker> ticker_;
    std::unique_ptr<Motor> motor_;
    std::unique_ptr<SegmentsExecutor<axesSize, Motor, Ticker>> executor_;
    std::unique_ptr<GCodeInterpreter<axesSize>> interpreter_;
    std::unique_ptr<GCodeParser<axesSize>> parser_;

    Ui::MainWindow ui_;
    QSerialPort *port_;
    QTimer *tickerTimer_;
    QTimer *statusTimer_;
};
