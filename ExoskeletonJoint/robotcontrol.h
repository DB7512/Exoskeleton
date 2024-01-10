#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <QObject>
#include <QTime>
#include <QTimer>
#include <QThread>
#include "serialportencoder.h"
#include "serialportmotor.h"

class RobotControl : public QObject
{
    Q_OBJECT
public:
    explicit RobotControl(QObject *parent = nullptr);


signals:
    void sig_startSerial();
    void sig_needData();

private:
    bool m_isStop;//robotcontrol开启标志
    bool m_motorStatus;//电机状态标志
    bool m_encoderStatus;//编码器状态标志
    bool m_torqueStatus;//扭矩传感器标志
    QSerialPort *m_serial;
    QThread *m_serialThread;
    QTimer *m_timer;
    // 串口
    SerialPortEncoder *m_encoderPort;
    SerialPortMotor *m_motorPort;


public slots:
    //robotcontrol
    void startThreadSlot();

    void handleEncoderData(uint32_t positionValue);
};

static RobotControl& GetRobotControlInstance()
{
    static RobotControl RobotControlInstance;
    return RobotControlInstance;
}

#endif // ROBOTCONTROL_H
