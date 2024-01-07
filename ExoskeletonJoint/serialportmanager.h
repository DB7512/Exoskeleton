#ifndef SERIALPORTMANAGER_H
#define SERIALPORTMANAGER_H

#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QTimer>

class SerialPortManager : public QObject
{
    Q_OBJECT
public:
    explicit SerialPortManager(QObject *parent = nullptr);
    ~SerialPortManager();

       void start();

   private slots:
       void handleMotorPortDataReceived(const QByteArray& data);
       void handleEncoderPortDataReceived(const QByteArray &data);
       void sendMotorPortData();
       void sendEncoderPortData();
       void startThreadSlot();

   private:
       QSerialPort m_serialPortMotor;
       QSerialPort m_serialPortEncoder;

       // 定义接收线程和发送线程
//       QThread *m_receiveThreadMotor;
//       QThread *m_sendThreadMotor;
//       QThread *m_receiveThreadEncoder;
//       QThread *m_sendThreadEncoder;

       QTimer *m_timerMotor;
       QTimer *m_timerEncoder;

       void setupPort(QSerialPort &port, const QString &portName, int baudRate);

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

};

#endif // SERIALPORTMANAGER_H
