#ifndef SERIALPORTMOTOR_H
#define SERIALPORTMOTOR_H

#include <QObject>
#include <QSerialPort>
#include <QTimer>
#include <QTime>
#include <QThread>

typedef enum{
    LOCK,           //锁定（Default）
    FOC,            //FOC闭环
    CALIBRATION,    //编码器校准
}MOTORMODE;       //电机工作模式

struct mmyMotorCmd{
    uint8_t Id;
    uint8_t Mode;
    float T;
    float W;
    float Pos;
    float K_P;
    float K_W;
    // QString crc;
};//主机控制协议

struct mmyMotorData{
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
    mmyMotorCmd m_motorCmd;
    mmyMotorData m_motorData;
    int numi;
    int numj;

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
    void run();
};

#endif // SERIALPORTMOTOR_H
