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
    bool portIsOpen(); // 判断串口是否打开
    void closePort();  // 关闭串口

  signals:
    void sendReadMsg(const QByteArray &);

  public slots:
    void init();
    void write(const QByteArray &); // 发送命令
    void read();                    // 读取数据

  private:
    QSerialPort *port = nullptr;
    bool potStatu = true;
    QByteArray readData;
};

#endif // SERIALPORT_H