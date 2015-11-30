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

    executor_.reset(new Exec(motor_, ticker_));
    interpreter_.reset(new Interp(executor_.get(), printer_));
    parser_.reset(new Parser(interpreter_.get()));

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
    ui_.pbRefresh->setEnabled(false);
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
    ui_.pbRefresh->setEnabled(true);
}

void MainWindow::on_pbResetPosition_clicked() {
    executor_->setPosition();
    interpreter_->printCurrentPosition();
    updatePositions();
}

void MainWindow::on_pbSendData_clicked() {
    auto data = ui_.teData->toPlainText();
    for (auto &&line : data.split("\n", QString::SkipEmptyParts)) {
        line += "\n";
        parser_->parseLine(line.toLatin1().constData());
    }
}

void MainWindow::readyRead() {
    while (port_->canReadLine()) {
        auto line = port_->readLine();
        if (line.isEmpty()) {
            continue;
        }
        qDebug() << ">>" << line.trimmed();
        parser_->parseLine(line.data());
    }
}

void MainWindow::statusTimerTimeout() {
    if (executor_->isRunning()) {
        interpreter_->printCurrentPosition();
    }
}

void MainWindow::executionStarted() {
    qDebug() << "Execution started.";
    statusTimer_->start();
}

void MainWindow::executionStopped() {
    qDebug() << "Execution stopped.";
    statusTimer_->stop();
    interpreter_->printCompleted();
}

void MainWindow::updatePositions() {
    QLabel *labels[] = {ui_.lA, ui_.lX, ui_.lY, ui_.lZ, ui_.lB};
    QLabel *labels2[] = {ui_.lA_2, ui_.lX_2, ui_.lY_2, ui_.lZ_2, ui_.lB_2};
    auto spu = interpreter_->stepsPerUnitLength();
    for (int i = 0; i < TestAxesTraits::size; i++) {
        auto step = executor_->position()[i];
        labels[i]->setText(QString::number(step));
        labels2[i]->setText(QString::number(step / spu[i]));
    }
}