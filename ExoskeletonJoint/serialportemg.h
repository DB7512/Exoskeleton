#ifndef SERIALPORTEMG_H
#define SERIALPORTEMG_H

#include <QObject>
#include <QSerialPort>
#include <QTimer>
#include <QTime>
#include <vector>

using namespace std;
class SerialPortEMG : public QObject
{
    Q_OBJECT
public:
    explicit SerialPortEMG(QObject *parent = nullptr);
    ~SerialPortEMG();

    //初始化EMG串口
    void InitEMGPort();
    //EMG串口状态标志
    bool m_serialEMGStatus;
    //EMG串口指针
    QSerialPort *m_serialEMG;
    vector<QString> m_EMGMessage;

signals:
    // //给robotcontrol信号以解析数据
    // void sig_getPositionData(uint32_t positionValue);
    // //向编码器发送指令信号
    // void sig_encoderDataRequest();

public slots:
    //接收EMG数据
    void ReceiveEMGData();


private:
    void parseEMGData(QByteArray& data);
    struct timeval tptime1,tptime2;
    // EMG定时器指针
    QTimer *m_EMGTimer;
    // 线程指针，暂不考虑使用
    QThread *m_thread;
    //延时函数ms
    void sleep(int msec);
};

#endif // SERIALPORTEMG_H
