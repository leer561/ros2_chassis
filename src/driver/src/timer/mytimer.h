#include <QObject>
#include <QTimer>

class MyTimer : public QObject
{
    Q_OBJECT

  public:
    MyTimer(QObject *parent);
    ~MyTimer();

  public slots:
    void TimerHandlerFunction();

  private:
    QTimer m_timer;
};