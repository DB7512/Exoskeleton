#include "encoderserialthread.h"
#include "robotcontrol.h"
#include <QTime>
#include <QCoreApplication>
#include <QDebug>
#include <QTimer>

EncoderSerialThread::EncoderSerialThread()
{
    stopped = false;
    serialStatus = false;
    serialPort = new QSerialPort;
    InitSerialPort();
//    QTimer *timer = new QTimer(this);
//    connect(timer,&QTimer::timeout,this,&EncoderSerialThread::sendData);
//    timer->start(20);//ms
    connect(serialPort,&QSerialPort::readyRead,this,&EncoderSerialThread::readData);
}

EncoderSerialThread::~EncoderSerialThread()
{

}

void EncoderSerialThread::InitSerialPort()
{
    if(serialPort->isOpen()) {
        serialPort->close();
    }
    // Set up serial port configurations
    serialPort->setPortName("/dev/ttyUSB1");
    serialPort->setBaudRate(115200);
    serialPort->setDataBits(QSerialPort::Data8);     //串口数据8bit
    serialPort->setParity(QSerialPort::NoParity);    //无奇偶校验位
    serialPort->setStopBits(QSerialPort::OneStop);   //1bit停止位
    serialPort->setFlowControl(QSerialPort::NoFlowControl);  //无流控制
    if (!serialPort->open(QIODevice::ReadWrite)) {
        serialStatus = false;
        qDebug("EncoderSerialport opening failed!");
    } else {
        serialStatus = true;
//        GetRobotControlInstance().m_encoderStatus = true;
        qDebug("EncoderSerialport opened successfully!");
    }
}

void EncoderSerialThread::sendData()
{
    QByteArray data;
    data.resize(8);
    data[0] = char(0xFF);
    data[1] = char(0x03);
    data[2] = char(0x00);
    data[3] = char(0x02);
    data[4] = char(0x00);
    data[5] = char(0x04);
    data[6] = char(0xF0);
    data[7] = char(0x17);
    serialPort->write(data);
    serialPort->flush();
}

void EncoderSerialThread::readData()
{
    QByteArray data = serialPort->readAll();
//    qDebug()<<data;
//    qDebug()<<data.toHex();
    //转换为字符串
    //QString serialBuffer = data.toHex();
    if(!data.isEmpty()) {
        uint8_t error = 0;
        uint8_t warning = 0;
        uint8_t errorbit = static_cast<uint8_t>(data.at(10));
        error |= errorbit >> 7;
        warning |= (errorbit >> 6) & 0x1;
        if(error != 0 && warning !=0) {
            //数据6bytes（48bits），大端模式，其中16bits（多圈计数）+17bits（位置数据）
            QByteArray positionData;        //存放6bytes数据
            positionData.append(data[7]);
            positionData.append(data[8]);
            positionData.append(data[5]);
            positionData.append(data[6]);
            positionData.append(data[3]);
            positionData.append(data[4]);
            //获取17bits位置数据（0-17bits）
            QByteArray positionBytes = positionData.right(3);
            uint32_t positionValue = 0;
            for (int i = 0; i < 3; ++i) {
                uint32_t byteValue = static_cast<uint8_t>(positionBytes.at(2-i));   //大端模式n-i，小端模式i
                positionValue |= byteValue << (8 * i);      //移位操作
            }
            positionValue &= 0x1FFFF;
            //获取16bits多圈计数（18-33bits）
            QByteArray counterBytes = positionData.mid(2,3);
            uint32_t counterValue = 0;
            for (int i = 0; i < 3; ++i) {
                uint32_t counterValue = static_cast<uint8_t>(counterBytes.at(2-i)); //大端模式n-i，小端模式i
                counterValue |= counterValue << (8 * i);      //移位操作
            }
            counterValue >>= 1;
            counterValue &= 0xFFFF;
//            qDebug()<<positionData;
//            qDebug()<<"counterValue"<<counterValue;
//            qDebug()<<"positionValue"<<positionValue;
        } else {
            if(error == 0) {
                qDebug("Encoder error!!!");
            }
            if(warning == 0) {
                qDebug("Encoder warning!!!");
            }
        }
    }
}

void EncoderSerialThread::run()
{
    int aaa = 1;
    while(!stopped) {
        if(serialStatus) {
            sendData();
            if(serialPort->waitForReadyRead(1)) {
                readData();
            }
            sleep(5);
        } else {
            stopped = true;
        }
    }
}

void EncoderSerialThread::sleep(int msec)
{
    QTime dieTime = QTime::currentTime().addMSecs(msec);
    while( QTime::currentTime() < dieTime )
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}
