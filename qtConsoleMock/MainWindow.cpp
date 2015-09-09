#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    ui_.setupUi(this);
    port_ = new QSerialPort(this);

    tickerTimer_ = new QTimer(this);
    tickerTimer_->setTimerType(Qt::PreciseTimer);
    tickerTimer_->setInterval(1);

    statusTimer_ = new QTimer(this);
    statusTimer_->setTimerType(Qt::PreciseTimer);
    statusTimer_->setInterval(200);

    printer_ = new SerialPrinter(port_);
    ticker_ = new Ticker(tickerTimer_);
    motor_ = new Motor(this);

    executor_.reset(new SegmentsExecutor<Motor, Ticker, TestAxesTraits>(motor_, ticker_));
    interpreter_.reset(new GCodeInterpreter<TestAxesTraits>(executor_.get(), printer_));
    parser_.reset(new GCodeParser<TestAxesTraits>(interpreter_.get()));

    executor_->setOnStarted([](void *p) { static_cast<This *>(p)->executionStarted(); }, this);
    executor_->setOnStopped([](void *p) { static_cast<This *>(p)->executionStopped(); }, this);
    interpreter_->setTicksPerSecond(1000u / tickerTimer_->interval());

    connect(port_, &QSerialPort::readyRead, this, &This::readyRead);
    connect(statusTimer_, &QTimer::timeout, this, &This::statusTimerTimeout);
    connect(motor_, &Motor::update, this, &This::updatePositions);

    on_pbRefresh_clicked();
}

MainWindow::~MainWindow() {}

void MainWindow::on_pbRefresh_clicked() {
    auto baudRates = QSerialPortInfo::standardBaudRates();
    ui_.cbBaudRate->clear();
    for (auto rate : baudRates) {
        ui_.cbBaudRate->addItem(QString::number(rate), rate);
    }
    ui_.cbBaudRate->setCurrentText("115200");

    auto portInfos = QSerialPortInfo::availablePorts();
    if (portInfos.empty()) {
        qDebug() << "No available ports.";
        return;
    }
    qDebug() << "Found " << portInfos.size() << " ports.";

    ui_.cbPorts->clear();
    for (auto const &info : portInfos) {
        if (!info.isBusy() && !info.isNull()) {
            ui_.cbPorts->addItem(info.portName());
        }
    }

    ui_.pbConnect->setEnabled(true);
}

void MainWindow::on_pbConnect_clicked() {
    if (port_->isOpen()) {
        qDebug() << "Port is already opened.";
        return;
    }

    auto portName = ui_.cbPorts->currentText();
    if (portName.isEmpty()) {
        qDebug() << "Nothing is selected.";
        return;
    }

    int baudRate = ui_.cbBaudRate->currentData().toInt();
    qDebug() << "Opening port " << portName << " with baud rate " << baudRate;

    port_->setPortName(portName);
    port_->setBaudRate(baudRate);

    if (!port_->open(QIODevice::ReadWrite)) {
        qDebug() << "Cannot open port: " << port_->errorString();
    }

    ui_.pbConnect->setEnabled(false);
    ui_.pbDisconnect->setEnabled(true);
}

void MainWindow::on_pbDisconnect_clicked() {
    if (!port_->isOpen()) {
        qDebug() << "Port is already closed.";
        return;
    }
    port_->close();
    qDebug() << "Port is closed.";

    ui_.pbConnect->setEnabled(true);
    ui_.pbDisconnect->setEnabled(false);
}

void MainWindow::on_pbReset_clicked() {}

void MainWindow::readyRead() {
    if (!port_->canReadLine()) {
        return;
    }
    auto line = port_->readLine();
    if (line.isEmpty()) {
        return;
    }
    qDebug() << ">>" << line.trimmed();

    parser_->parseLine(line.data());
}

void MainWindow::statusTimerTimeout() {
    if (executor_->isRunning() && !executor_->isHoming()) {
        interpreter_->printCurrentPosition();
    }
}

void MainWindow::executionStarted() {
    qDebug() << "Execution started.";

    QDial *dials[] = {ui_.dA, ui_.dX, ui_.dY, ui_.dZ, ui_.dB};
    for (int i = 0; i < TestAxesTraits::size; i++) {
        auto dial = dials[i];
        auto maxUnits = interpreter_->maxDistance()[i];
        if (isfinite(maxUnits)) {
            auto spu = interpreter_->stepsPerUnitLength()[i];
            auto maxSteps = maxUnits * spu;
            dial->setMaximum(maxSteps);
        } else {
            dial->setWrapping(true);
        }
    }

    statusTimer_->start();
}

void MainWindow::executionStopped() {
    qDebug() << "Execution stopped.";
    statusTimer_->stop();
    interpreter_->printCompleted();
}

void MainWindow::updatePositions() {
    QDial *dials[] = {ui_.dA, ui_.dX, ui_.dY, ui_.dZ, ui_.dB};
    for (int i = 0; i < TestAxesTraits::size; i++) {
        auto dial = dials[i];
        auto step = executor_->position()[i];
        dial->setValue(step);
    }
}