#include "serialport.h"
#include <QSerialPort>
#include <QString>
#include <QDebug>

serialport::serialport()
{
}

serialport::~serialport()
{
    delete port;
}

// 打开串口
QSerialPort *serialport::InitSerialPort(QString portName, int baudRate)
{
    //对串口进行一些初始化
    port = new QSerialPort();
    port->setPortName(portName); // 串口名
    port->open(QIODevice::ReadWrite);
    port->setBaudRate(baudRate);                      //波特率
    port->setDataBits(QSerialPort::Data8);            //数据字节，8字节
    port->setParity(QSerialPort::NoParity);           //校验，无
    port->setFlowControl(QSerialPort::NoFlowControl); //数据流控制,无
    port->setStopBits(QSerialPort::OneStop);          //一位停止位

    return port;
}

// 串口运行状态
bool serialport::PortIsOpen()
{
    // 如果对象port还没实例化
    if (port == NULL)
        return false;

    return port->isOpen();
}

// 发送字符串
void serialport::SendMsgToPort(QString hex)
{
    if (!port->isOpen())
    {
        qDebug() << "not open";
        return;
    }

    port->write(hex.toLatin1());
}
// 关闭串口
void serialport::ClosePort()
{
    port->close();
}