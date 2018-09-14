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
    qDebug() << "cmd data" << data;
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

// 接收读取的数据
void SerialController::getReadMsg(const QByteArray &data)
{
    // 读取判断数据 长度小于14不读
    if (data.size() < 14)
        return;
    // 转换为 unsigned char
    unsigned char *buffer = (unsigned char *)data.constData();
    std::vector<unsigned char> bufferToCompress(data.begin(), data.end());

    // 判断头部 不是0xEA 即234 返回
    if (data[0] != 234)
        return;

    // 编码器值
    encoderData = {data[11], data[12]};
    publishOdometry(encoderData);
}

// 处理编码器 派生类重写
void SerialController::publishOdometry(const std::vector<int> &data)
{
}