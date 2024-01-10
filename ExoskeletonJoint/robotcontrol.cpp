#include "robotcontrol.h"
#include <QDebug>
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

}

void RobotControl::startThreadSlot()
{
    qDebug("111");
    while(!m_isStop) {
        QThread::sleep(2);
    }
}

void RobotControl::handleEncoderData(uint32_t positionValue)
{
    qDebug()<<"positionValue"<<positionValue;
}
