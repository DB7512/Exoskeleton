#ifndef SERIALPORTENCODER_H
#define SERIALPORTENCODER_H

#include <QObject>
#include <QSerialPort>
#include <QTimer>
#include <QTime>

class SerialPortEncoder : public QObject
{
    Q_OBJECT
public:
    explicit SerialPortEncoder(QObject *parent = nullptr);
    ~SerialPortEncoder();
    //初始化编码器串口
    void InitEncoderPort();
    //编码器串口状态标志
    bool m_serialEncoderStatus;
    //编码器串口指针
    QSerialPort *m_serialEncoder;

signals:
    //给robotcontrol信号以解析数据
    void sig_getPositionData(uint32_t positionValue);
    //向编码器发送指令信号
    void sig_encoderDataRequest();

public slots:
    //向编码器发送查询指令
    void SendDataToEncoder();
    //接收编码器数据
    void ReceiveDataFromEncoder();


private:
    void parseEncoderData(QByteArray& data);
    struct timeval tptime1,tptime2;
    //编码器定时器指针
    QTimer *m_encoderTimer;
    //线程指针，暂不考虑使用
    QThread *m_thread;
    //延时函数ms
    void sleep(int msec);

};
static SerialPortEncoder* GetEncoderPortInstance()
{
    static SerialPortEncoder EncoderPortInstance;
    return &EncoderPortInstance;
}
#endif // SERIALPORTENCODER_H
