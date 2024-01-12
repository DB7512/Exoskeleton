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
    m_EMGPort = new SerialPortEMG;
}

RobotControl::~RobotControl()
{

}

void RobotControl::startThreadSlot()
{
    qDebug("111");
    while(!m_isStop) {
        QString data = "111,121,1341,123,1245,12367\r\n1234,13,";
        vector<int> EMGData(6);
        QStringList list = data.split("\r\n");
        QString tempStr;
        if (list.size() > 0) {
            for(int i = 0; i < list.size() - 1; ++i) {
                tempStr = list[i];
                QStringList listData = tempStr.split(",");
                for(int j = 0; j < listData.size(); ++j) {
                    tempStr = listData[j];
                    EMGData[j] = tempStr.toInt();
                }
            }
        }
    }
}

void RobotControl::handleEncoderData(uint32_t positionValue)
{
    qDebug()<<"positionValue"<<positionValue;
}
