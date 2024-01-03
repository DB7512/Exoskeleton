#include "robotcontrol.h"

RobotControl::RobotControl(QObject *parent)
    : QObject{parent}
{
    m_isStop = false;
    m_motorStatus = false;
    m_encoderStatus = false;
    m_torqueStatus = false;
}

void RobotControl::startThreadSlot()
{
    while(!m_isStop) {
        if(!m_motorStatus) {
            motorSerial = new MotorSerialThread;
            m_motorStatus = true;
        }
        if(!m_encoderStatus) {
            encoderSerial = new EncoderSerialThread;
            m_encoderStatus = true;
        }
        motorSerial->start();
        encoderSerial->start();
    }

}
