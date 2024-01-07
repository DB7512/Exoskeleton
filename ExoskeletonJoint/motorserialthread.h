#ifndef MOTORSERIALTHREAD_H
#define MOTORSERIALTHREAD_H

#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QThread>

typedef enum{
    LOCK,
    FOC,
    CALIBRATION,
}MOTORSTATUS;

struct ControlProtocol{
    QString head;
    int motor_id;
    MOTORSTATUS status;
    int tau_set;
    int omega_set;
    int theta_set;
    int k_pose;
    int k_spd;
    QString crc;
};//主机控制协议

struct FeedbackData{
    QString head;
    int motor_id;
    MOTORSTATUS status;
    int tau_fbk;
    int omega_fbk;
    int theta_fbk;
    int8_t temp;
    int force;
    QString crc;
};//电机反馈数据

class MotorSerialThread : public QThread
{
    Q_OBJECT
public:
    MotorSerialThread(QObject *parent = nullptr);
    ~MotorSerialThread();

    //初始化串口
    void InitSerialPort();
    //组合主机控制协议
    void SendAgreementContent(QByteArray& serialdata, uint8_t motorid, uint8_t status,
                  int16_t tua, int16_t omega, int32_t theta, int16_t kpos, int16_t kspd);
    //控制参数设置
    int16_t GetTuaSet(float tua);
    int16_t GetOmegaSet(float omega);
    int32_t GetThetaSet(float theta);
    int16_t GetKpos(float kp);
    int16_t GetKspd(float kw);
    //CRC16-CCITT校验
    uint16_t CRC16CCITT(const QByteArray &data, int len);

    void sendData();
    void readData();



protected:

signals:

private:
    QSerialPort *serialPort;
    ControlProtocol sendStruct;
    FeedbackData receiveStruct;
    bool stopped;
    bool serialStatus;
private:
    void run();
    void sleep(int msec);
};

#endif // MOTORSERIALTHREAD_H
