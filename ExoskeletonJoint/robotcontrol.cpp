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
    // 电机控制
    m_motorControl = new MotorControl;
    // 编码器数据
    m_encoderPort = new SerialPortEncoder;
    connect(m_encoderPort,&SerialPortEncoder::sig_getPositionData,this,&RobotControl::handleEncoderData);
    // sEMGs数据
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
    // 开启电机线程
    m_motorControl->start();
    while(!m_isStop) {
        sleep(1000);
    }
}

void RobotControl::handleEncoderData(uint32_t positionValue)
{
    qDebug()<<"positionValue"<<positionValue;
}
