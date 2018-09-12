#include "serial-port.h"
#include <QByteArray>
#include <QDataStream>
#include <QDebug>
#include <QSerialPort>

SerialPort::SerialPort()
{
}

SerialPort::~SerialPort() { delete port; }

void SerialPort::init()
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
    port->setReadBufferSize(23);

    // 绑定readyread
    connect(port, &QSerialPort::readyRead, this, &SerialPort::read);
}

// 串口运行状态
bool SerialPort::portIsOpen()
{
    // 如果对象port还没实例化
    if (port == NULL)
        return false;

    return port->isOpen();
}

// 发送命令
void SerialPort::write(const QByteArray &cmd)
{
    if (!port->isOpen())
    {
        qDebug() << "port not open";
    }
    port->write(cmd);
}

// 发送命令
void SerialPort::read()
{
    readData = port->readAll();
    qDebug() << "readData" << readData;
    QDataStream dataStream(readData);
    quint16 result;
    dataStream >> result;
    qDebug() << "readData 16bit" << result;
    emit sendReadMsg(readData);
}

// 关闭串口
void SerialPort::closePort() { port->close(); }