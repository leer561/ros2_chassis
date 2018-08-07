#include "mytimer.h"
#include <QDebug>
#include <QTimer>
#include <QObject>

MyTimer::MyTimer(QObject *parent) : QObject(parent)
{
    // connect the timer's timeout to our TimerHandlerFunction
    connect(&m_timer, SIGNAL(timout()), this, SLOT(TimerHandlerFunction()));
}
MyTimer::~MyTimer()
{
}

void MyTimer::TimerHandlerFunction()
{
    qDebug() << "mytimer";
}