#pragma once

#include "../include/sc/GCodeInterpreter.h"
#include "../include/sc/GCodeParser.h"
#include "../include/sc/SegmentsExecutor.h"

#include <QDebug>
#include <QMainWindow>
#include <QTimer>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <functional>
#include <memory>

#include "GeneratedFiles/ui_MainWindow.h"

using namespace StepperControl;

struct TestAxesTraits {
    static const int size = 5;
    static const char *names() { return "AXYZB"; }
};

class Motor : public QObject {
    Q_OBJECT
  public:
    explicit Motor(QObject *parent) : QObject(parent) {}

    template <unsigned i>
    void writeDirection(StepperNumber<i>, bool reverse) {}

    template <unsigned i>
    void writeStep(StepperNumber<i>, bool edge) {}

    static bool checkEndSwitchHit(size_t i) { return true; }

    static void begin() {}

    void end() { emit update(); }

  signals:
    void update();
};

class Ticker : public QObject {
    Q_OBJECT

  public:
    explicit Ticker(QTimer *parent) : QObject(parent), timer_(parent) {
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

class SerialPrinter : public QObject, public Printer {
    Q_OBJECT

  public:
    explicit SerialPrinter(QSerialPort *port) : QObject(port), port_(port) {}
    void print(const float *n, int size) override {
        for (int i = 0; i < size; i++) {
            write(n[i]);
            write(sep(i, size));
        }
    }
    void print(const int32_t *n, int size) override {
        for (int i = 0; i < size; i++) {
            write(n[i]);
            write(sep(i, size));
        }
    }
    void print(const char *n) override { write(n); }

    template <typename T>
    void write(T n) {
        QByteArray data;
        QTextStream(&data) << n;
        port_->write(data);
    }

    QSerialPort *port_;
};

class MainWindow : public QMainWindow {
    Q_OBJECT

  public:
    using This = MainWindow;
    using Exec = SegmentsExecutor<Motor, Ticker, TestAxesTraits>;
    using Interp = GCodeInterpreter<Exec, TestAxesTraits>;
    using Parser = GCodeParser<Interp, TestAxesTraits>;

    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

  public slots:
    void on_pbRefresh_clicked();
    void on_pbConnect_clicked();
    void on_pbDisconnect_clicked();
    void on_pbResetPosition_clicked();
    void on_pbSendData_clicked();

    void readyRead();
    void statusTimerTimeout();
    void executionStarted();
    void executionStopped();

    void updatePositions();

  private:
    SerialPrinter *printer_;
    Ticker *ticker_;
    Motor *motor_;
    std::unique_ptr<Exec> executor_;
    std::unique_ptr<Interp> interpreter_;
    std::unique_ptr<Parser> parser_;

    Ui::MainWindow ui_;
    QSerialPort *port_;
    QTimer *tickerTimer_;
    QTimer *statusTimer_;
};
