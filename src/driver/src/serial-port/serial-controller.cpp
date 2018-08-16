// 端口管理
#include "./serial-controller.h"
#include "./serial-port.h"

#include <vector>
#include <QThread>
#include <QByteArray>
#include <QDebug>

SerialController::SerialController(){};
SerialController::~SerialController()
{
    workerThread.quit();
    workerThread.wait();
};
void SerialController::SendMsgToPort(const std::vector<int> &cmd)
{
    // 转换cmd数据为QByteArray
    QByteArray data;
    int size = cmd.size();
    data.resize(size);
    for (int i = 0; i < size; i++)
    {
        data[i] = cmd[i];
    };
    qDebug() << "cmd data" << data;
    emit write(data);
};
void SerialController::init()
{
    SerialPort *port = new SerialPort;
    port->moveToThread(&workerThread);
    connect(&workerThread, &QThread::finished, port, &QObject::deleteLater);
    connect(this, &SerialController::write, port, &SerialPort::SendMsgToPort);
    workerThread.start();
}