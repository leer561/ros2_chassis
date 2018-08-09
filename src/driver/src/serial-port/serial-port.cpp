#include "serial-port.h"
#include <QByteArray>
#include <QDebug>
#include <QSerialPort>

SerialPort::SerialPort()
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

SerialPort::~SerialPort() { delete port; }

// 串口运行状态
bool SerialPort::PortIsOpen()
{
    // 如果对象port还没实例化
    if (port == NULL)
        return false;

    return port->isOpen();
}

// 发送命令
char SerialPort::SendMsgToPort(const int *cmd, const int size)
{
    // 转换cmd数据为QByteArray
    QByteArray data;
    data.resize(size);
    for (int i = 0; i < size; i++)
    {
        data[i] = *(cmd + i);
    }
    qDebug() << "cmd data" << data;
    // QByteArray ba;
    // ba.resize(5);
    // ba[0] = 0xea;
    // ba[1] = 0x03;
    // ba[2] = 0x50;
    // ba[3] = 0x00;
    // ba[4] = 0x0d;
    // qDebug() << "ba ba" << ba;

    if (!port->isOpen())
    {
        qDebug() << "port not open";
        return -1;
    }
    return port->write(data);
}
// 关闭串口
void SerialPort::ClosePort() { port->close(); }