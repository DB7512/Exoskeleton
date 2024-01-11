#ifndef SERIALPORTMOTOR_H
#define SERIALPORTMOTOR_H

#include <QObject>
#include <QSerialPort>
#include <QTimer>
#include <QTime>

typedef enum{
    LOCK,           //锁定（Default）
    FOC,            //FOC闭环
    CALIBRATION,    //编码器校准
}MOTORSTATUS;       //电机工作模式

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

class SerialPortMotor : public QObject
{
    Q_OBJECT
public:
    explicit SerialPortMotor(QObject *parent = nullptr);
    ~SerialPortMotor();

    //初始化电机串口
    void InitMotorPort();
    bool m_serialMotorStatus;
    //电机串口指针
    QSerialPort *m_serialMotor;
signals:
    void sig_sendMotorData();

public slots:
    //向电机发送查询指令
    void SendDataToMotor();
    //接收电机数据
    void ReceiveDataFromMotor();
private:
    QTimer *m_motorTimer;

    void SendAgreementContent(QByteArray& serialdata, uint8_t motorid, uint8_t status,
                              int16_t tua, int16_t omega, int32_t theta, int16_t kpos, int16_t kspd);
    //控制参数设置
    int16_t GetTuaSet(float tua);
    int16_t GetOmegaSet(float omega);
    int32_t GetThetaSet(float theta);
    int16_t GetKpos(float kp);
    int16_t GetKspd(float kw);
    //CRC16-CCITT校验
    uint16_t CRC16CCITT(const QByteArray &data);
    void parseMotorData(QByteArray& data);
    //延时函数ms
    void sleep(int msec);
};

#endif // SERIALPORTMOTOR_H
