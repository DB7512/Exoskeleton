#include "serialportencoder.h"
#include <QDebug>
#include <QThread>
#include <QTime>
#include <QCoreApplication>
#include <sys/time.h>

SerialPortEncoder::SerialPortEncoder(QObject *parent)
    : QObject{parent}
{
    m_serialEncoder = new QSerialPort;
    InitEncoderPort();
    connect(this,SIGNAL(sig_encoderDataRequest()),this,SLOT(SendDataToEncoder()));
    connect(m_serialEncoder,SIGNAL(readyRead()),this,SLOT(ReceiveDataFromEncoder()));
}

SerialPortEncoder::~SerialPortEncoder()
{
    if(m_serialEncoder) {
        m_serialEncoder->close();
        delete m_serialEncoder;
    }
}

void SerialPortEncoder::InitEncoderPort()
{
    if(m_serialEncoder->isOpen()) {
        m_serialEncoder->close();
    }
    // Setup serial port configurations
    m_serialEncoder->setPortName("/dev/ttyUSB0");
    m_serialEncoder->setBaudRate(115200);
    m_serialEncoder->setDataBits(QSerialPort::Data8);     //串口数据8bit
    m_serialEncoder->setParity(QSerialPort::NoParity);    //无奇偶校验位
    m_serialEncoder->setStopBits(QSerialPort::OneStop);   //1bit停止位
    m_serialEncoder->setFlowControl(QSerialPort::NoFlowControl);  //无流控制
    if (!m_serialEncoder->open(QIODevice::ReadWrite)) {
        m_serialEncoderStatus = false;
        qDebug("EncoderSerialport opening failed!");
    } else {
        m_serialEncoderStatus = true;
        qDebug("EncoderSerialport opened successfully!");
        //首次发送
        SendDataToEncoder();
    }
}

void SerialPortEncoder::SendDataToEncoder()
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
    // qDebug()<<"子线程槽函数发送数据："<<data<<"string"<<data.toHex()<<"线程ID："<<QThread::currentThread();
    m_serialEncoder->write(data);
    m_serialEncoder->flush();
    gettimeofday(&tptime1,NULL);
}

void SerialPortEncoder::ReceiveDataFromEncoder()
{
    QByteArray data = m_serialEncoder->readAll();
    if(data.size() > 0) {
        parseEncoderData(data);
        // 触发向编码器发送指令函数
        emit sig_encoderDataRequest();
    }
    // gettimeofday(&tptime2,NULL);
    // float timeuse = (1000000*(tptime2.tv_sec-tptime1.tv_sec) + tptime2.tv_usec-tptime1.tv_usec);
    // qDebug()<<"time"<<timeuse;
    // QString dataStr = data.toHex();
    // qDebug()<<"子线程收到数据："<<dataStr<<"线程ID："<<QThread::currentThread();
}

void SerialPortEncoder::parseEncoderData(QByteArray &data)
{
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
        // qDebug()<<positionData;
        // qDebug()<<"counterValue"<<counterValue;
        // qDebug()<<"positionValue"<<positionValue;
    } else {
        if(error == 0) {
            qDebug("Encoder error!!!");
        }
        if(warning == 0) {
            qDebug("Encoder warning!!!");
        }
    }
    // 触发robotcontrol解析数据函数
    // emit sig_getPositionData(positionValue);
    // sleep(1);
}

void SerialPortEncoder::sleep(int msec)
{
    QTime dieTime = QTime::currentTime().addMSecs(msec);
    while( QTime::currentTime() < dieTime )
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}
