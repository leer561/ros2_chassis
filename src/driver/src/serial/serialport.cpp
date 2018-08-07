#include "serialport.h"
#include <QSerialPort>
#include <QString>
#include <QDebug>

Serialport::Serialport()
{
    //对串口进行一些初始化
    port = new QSerialPort();
    port->setPortName("/dev/ttyUSB0"); // 串口名
    port->open(QIODevice::ReadWrite);
    port->setBaudRate(QSerialPort::Baud38400);        //波特率
    port->setDataBits(QSerialPort::Data8);            //数据字节，8字节
    port->setParity(QSerialPort::NoParity);           //校验，无
    port->setFlowControl(QSerialPort::NoFlowControl); //数据流控制,无
    port->setStopBits(QSerialPort::OneStop);          //一位停止位
}

Serialport::~Serialport()
{
    delete port;
}

// 串口运行状态
bool Serialport::PortIsOpen()
{
    // 如果对象port还没实例化
    if (port == NULL)
        return false;

    return port->isOpen();
}

// 发送字符串
void Serialport::SendMsgToPort(QString hex)
{
    if (!port->isOpen())
    {
        qDebug() << "not open";
        return;
    }
    qDebug() << "port open mode" << port->openMode();
    auto data = port->write(hex.toLatin1());
    qDebug() << "write data " << data;
}
// 关闭串口
void Serialport::ClosePort()
{
    port->close();
}