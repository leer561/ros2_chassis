#ifndef SERIALPORT_H
#define SERIALPORT_H

// 端口管理
#include <QSerialPort>
class SerialPort
{
  public:
    SerialPort();
    ~SerialPort();

    bool PortIsOpen();                          // 判断串口是否打开
    char SendMsgToPort(const int *, const int); // 发送命令
    void ClosePort();                           // 关闭串口

  private:
    QSerialPort *port = nullptr;
    bool potStatu = true;
    char buf[80];
};

#endif // SERIALPORT_H