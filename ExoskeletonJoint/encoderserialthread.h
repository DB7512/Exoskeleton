#ifndef ENCODERSERIALTHREAD_H
#define ENCODERSERIALTHREAD_H

#include <QThread>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

class EncoderSerialThread : public QThread
{
public:
    EncoderSerialThread();
    ~EncoderSerialThread();
    //初始化串口
    void InitSerialPort();
    void sendData();
    void readData();


    QSerialPort *serialPort;

protected:

signals:

private:
    bool stopped;
    bool serialStatus;
private:
    void run();
    void sleep(int msec);
};

#endif // ENCODERSERIALTHREAD_H
