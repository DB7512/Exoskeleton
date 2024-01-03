#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <QObject>
#include "motorserialthread.h"
#include "encoderserialthread.h"

class RobotControl : public QObject
{
    Q_OBJECT
public:
    explicit RobotControl(QObject *parent = nullptr);
    MotorSerialThread *motorSerial;
    EncoderSerialThread *encoderSerial;
signals:

private:
    bool m_isStop;//robotcontrol开启标志
    bool m_motorStatus;//电机状态标志
    bool m_encoderStatus;//编码器状态标志
    bool m_torqueStatus;//扭矩传感器标志

public slots:
    void startThreadSlot();
};

static RobotControl& GetRobotControlInstance()
{
    static RobotControl RobotControlInstance;
    return RobotControlInstance;
}

#endif // ROBOTCONTROL_H
