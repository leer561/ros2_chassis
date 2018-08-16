#ifndef SERIALPORT_H
#define SERIALPORT_H

// 端口管理
#include <QByteArray>
#include <QSerialPort>
#include <QObject>

class SerialPort : public QObject
{
    Q_OBJECT
  public:
    SerialPort();
    ~SerialPort();

    bool PortIsOpen(); // 判断串口是否打开
    void ClosePort();  // 关闭串口
  public slots:
    void SendMsgToPort(const QByteArray &); // 发送命令

  private:
    QSerialPort *port = nullptr;
    bool potStatu = true;
    char buf[80];
};

#endif // SERIALPORT_H