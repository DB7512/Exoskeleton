#include "robotcontrol.h"
#include <QDebug>
#include <QCoreApplication>

using namespace std;
RobotControl::RobotControl(QObject *parent)
    : QObject{parent}
{
    m_isStop = false;
    m_motorStatus = false;
    m_encoderStatus = false;
    m_torqueStatus = false;

    m_encoderPort = new SerialPortEncoder;
    connect(m_encoderPort,&SerialPortEncoder::sig_getPositionData,this,&RobotControl::handleEncoderData);
    m_motorPort = new SerialPortMotor;
    m_EMGPort = new SerialPortEMG;
}

RobotControl::~RobotControl()
{

}

void RobotControl::sleep(int msec)
{
    QTime dieTime = QTime::currentTime().addMSecs(msec);
    while( QTime::currentTime() < dieTime )
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void RobotControl::startThreadSlot()
{
    qDebug("111");
    while(!m_isStop) {
        sleep(1000);
    }
}

void RobotControl::handleEncoderData(uint32_t positionValue)
{
    qDebug()<<"positionValue"<<positionValue;
}
