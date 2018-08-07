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
void Serialport::SendMsgToPort()
{
    if (!port->isOpen())
    {
        qDebug() << "not open";
        return;
    }
    qDebug() << "port open mode" << port->openMode();
    QByteArray ba;
    ba.resize(5);
    ba[0] = 0xea;
    ba[1] = 0x03;
    ba[2] = 0x50;
    ba[3] = 0x00;
    ba[4] = 0x0d;
    auto data = port->write(ba);
    qDebug() << "write data " << data;
    QString qstr = "Hello";
    port->write(qstr.toLatin1());
}
// 关闭串口
void Serialport::ClosePort()
{
    port->close();
}