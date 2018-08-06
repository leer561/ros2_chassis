#ifndef SERIALPORT_H
#define SERIALPORT_H

// 端口管理
#include <QSerialPort>
#include <QString>
class serialport
{
  public:
    serialport();
    ~serialport();

    QSerialPort *InitSerialPort(QString, int); // 初始化串口

    bool PortIsOpen();           // 判断串口是否打开
    void SendMsgToPort(QString); // 发送字符串
    void ClosePort();            // 关闭串口

  private:
    QSerialPort *port = NULL;
    bool potStatu = true;
    char buf[80];
};

#endif // SERIALPORT_H