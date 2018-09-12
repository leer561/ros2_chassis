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
}

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
    qDebug() << "cmd data" << cmd;
    emit write(data);
}

void SerialController::init()
{
    SerialPort *port = new SerialPort;
    port->moveToThread(&workerThread);
    connect(&workerThread, &QThread::finished, port, &QObject::deleteLater);
    connect(this, &SerialController::write, port, &SerialPort::write);
    connect(this, &SerialController::start, port, &SerialPort::init);
    connect(port, &SerialPort::sendReadMsg, this, &SerialController::getReadMsg);
    workerThread.start();
    emit start();
}

//接收读取的数据
void SerialController::getReadMsg(const QByteArray &data)
{
    qDebug() << "getReadMsg data" << data;
    // 读取判断数据 长度小于14不读
    if (data.size() < 14)
        return;
    // 判断头部 不是0xEA 返回
    if (data[0] != 'xEA')
        return;
}
