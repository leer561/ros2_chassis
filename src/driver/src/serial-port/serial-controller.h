#ifndef SERIALCONTROLLER_H
#define SERIALCONTROLLER_H

// 端口管理
#include <vector>
#include <QObject>
#include <QThread>
#include <QByteArray>

class SerialController : public QObject
{
    Q_OBJECT
    QThread workerThread;

  public:
    SerialController();
    ~SerialController();
    void SendMsgToPort(const std::vector<int> &); // 外部使用发送串口数据
    void init();                                  //初始化串口

  signals:
    void write(const QByteArray &);
};
#endif // SERIALCONTROLLER_H