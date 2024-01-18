#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <QObject>
#include <QThread>
#include "./include/serialPort/SerialPort.h"
#include "./include/unitreeMotor/unitreeMotor.h"

struct myMotorCmd{
    uint8_t Id;
    uint8_t Mode;
    float T;
    float W;
    float Pos;
    float K_P;
    float K_W;
    // QString crc;
};//主机控制协议

struct myMotorData{
    uint8_t Id;
    uint8_t Mode;
    float T;
    float W;
    float Pos;
    int8_t Temp;
    uint8_t MError;
    uint16_t Force;
    // QString crc;
};//电机反馈数据

class MotorControl : public QThread
{
    Q_OBJECT
public:
    MotorControl();
    ~MotorControl();

    void stopMode();
    myMotorCmd m_motorCmd;
    myMotorData m_motorData;
    bool m_isStop;
    bool m_motorStatus;
    int m_motorError;
private:
    void run();

    SerialPort  *m_serial;
    MotorCmd    *m_cmd;
    MotorData   *m_data;
    float m_lastPose;

signals:
};

#endif // MOTORCONTROL_H
