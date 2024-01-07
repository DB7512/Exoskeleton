#include "robotcontrol.h"
#include "serialportmanager.h"
RobotControl::RobotControl(QObject *parent)
    : QObject{parent}
{
    m_isStop = false;
    m_motorStatus = false;
    m_encoderStatus = false;
    m_torqueStatus = false;
    m_serialPortStatus = false;
}

void RobotControl::startThreadSlot()
{
    while(!m_isStop) {
        if(!m_serialPortStatus) {
            SerialPortManager serialPortManager;
            serialPortManager.start();
            m_serialPortStatus = true;
        }
    }
}
