#include "motorserialthread.h"
#include <math.h>
#include <QDebug>
#include <QTime>
#include <QCoreApplication>

MotorSerialThread::MotorSerialThread(QObject *parent)
    : QThread{parent}
{
    stopped = false;
    serialPort = new QSerialPort();
    InitSerialPort();
    QByteArray data;
    SendAgreementContent(data, 0, 0, GetTuaSet(0.0), GetOmegaSet(0.0), GetThetaSet(0), GetKpos(0), GetKspd(0.0));
    if(serialPort->waitForReadyRead(1)) {
        readData();
    }
    sleep(1000);
}

MotorSerialThread::~MotorSerialThread()
{
    if(serialPort->isOpen()) {
        serialPort->clear();
        serialPort->close();
    }
    delete serialPort;
}

void MotorSerialThread::InitSerialPort()
{
    if(serialPort->isOpen()) {
        serialPort->close();
    }
    // Set up serial port configurations
    serialPort->setPortName("/dev/ttyUSB0");
    serialPort->setBaudRate(4000000);
    serialPort->setDataBits(QSerialPort::Data8);     //串口数据8bit
    serialPort->setParity(QSerialPort::NoParity);    //无奇偶校验位
    serialPort->setStopBits(QSerialPort::OneStop);   //1bit停止位
    serialPort->setFlowControl(QSerialPort::NoFlowControl);  //无流控制
    if (!serialPort->open(QIODevice::ReadWrite)) {
        serialStatus = false;
        qDebug("Serialport opening failed!");
    } else {
        serialStatus = true;
        qDebug("Serialport opened successfully!");
    }
}

void MotorSerialThread::SendAgreementContent(QByteArray& serialdata, uint8_t motorid, uint8_t status,
                                             int16_t tua, int16_t omega, int32_t theta, int16_t kpos, int16_t kspd)
{
    if(serialdata.length() > 0) {
        serialdata.clear();
    }
    serialdata.append(0xFE);
    serialdata.append(0xEE);

    //模式设置 1byte
    int8_t lowNibble = static_cast<uint8_t>(motorid) & 0x0F;
    int8_t highNibble = static_cast<int8_t>(status) << 4;
    int8_t mode = highNibble | lowNibble;
    serialdata.append(mode);

    /*控制参数*/
    //tua 2bytes
    int8_t highByteTua = static_cast<int8_t>((tua >> 8) & 0xFF);
    int8_t lowByteTua = static_cast<int8_t>(tua & 0xFF);
    serialdata.append(lowByteTua);
    serialdata.append(highByteTua);

    //omega 2bytes
    int8_t highByteOmega = static_cast<int8_t>((omega >> 8) & 0xFF);
    int8_t lowByteOmega = static_cast<int8_t>(omega & 0xFF);
    serialdata.append(lowByteOmega);
    serialdata.append(highByteOmega);

    //theta 4bytes
    QByteArray thetaData;
    thetaData.resize(4);
    int8_t byte1 = static_cast<int8_t>((theta >> 24) & 0xFF); // 最高字节
    int8_t byte2 = static_cast<int8_t>((theta >> 16) & 0xFF); // 次高字节
    int8_t byte3 = static_cast<int8_t>((theta >> 8) & 0xFF);  // 次低字节
    int8_t byte4 = static_cast<int8_t>(theta & 0xFF);         // 最低字节
    thetaData[0] = byte4;
    thetaData[1] = byte3;
    thetaData[2] = byte2;
    thetaData[3] = byte1;
    serialdata.append(thetaData);

    //kpos 2bytes
    int8_t highByteKpos = static_cast<int8_t>((kpos >> 8) & 0xFF);
    int8_t lowByteKpos = static_cast<int8_t>(kpos & 0xFF);
    serialdata.append(highByteKpos);
    serialdata.append(lowByteKpos);

    //kspd 2bytes
    int8_t highByteKspd = static_cast<int8_t>((kspd >> 8) & 0xFF);
    int8_t lowByteKspd = static_cast<int8_t>(kspd & 0xFF);
    serialdata.append(lowByteKspd);
    serialdata.append(highByteKspd);

    //crc 2bytes
    uint16_t crc = CRC16CCITT(serialdata);
    int8_t highByteCRC = static_cast<int8_t>((crc >> 8) & 0xFF);
    int8_t lowByteCRC = static_cast<int8_t>(crc & 0xFF);
    serialdata.append(lowByteCRC);
    serialdata.append(highByteCRC);
    qDebug()<<"motorsenddata"<<serialdata.toHex();
    //发送
    serialPort->write(serialdata);
    serialPort->flush();
}

int16_t MotorSerialThread::GetTuaSet(float tua)
{
    if(fabs(tua) <= 127.99) {
        return (int16_t)(tua * 256);
    } else {
        return static_cast<int16_t>(0.0);
    }
}

int16_t MotorSerialThread::GetOmegaSet(float omega)
{
    if(fabs(omega) <= 804.0) {
        return static_cast<int16_t>(omega * 256 / 2 / M_PI);
    } else {
        return static_cast<int16_t>(0.0);
    }
}

int32_t MotorSerialThread::GetThetaSet(float theta)
{
    if(fabs(theta) <= 411774.0) {
        return static_cast<int32_t>(theta * 32768 / 2 / M_PI);
    } else {
        return static_cast<int32_t>(0.0);
    }
}

int16_t MotorSerialThread::GetKpos(float kp)
{
    if(kp >= 0 && kp <= 25.599) {
        return static_cast<int16_t>(kp * 1280);
    } else {
        return static_cast<int16_t>(0.0);
    }
}

int16_t MotorSerialThread::GetKspd(float kw)
{
    if(kw >= 0 && kw <= 25.599) {
        return static_cast<int16_t>(kw * 1280);
    } else {
        return static_cast<int16_t>(0.0);
    }
}

uint16_t MotorSerialThread::CRC16CCITT(const QByteArray &data)
{
    // CRC16_CCITT 计算方法
    //初始值
    uint16_t crc = 0x0000;
    int len = data.size();
    for(int j = 0; j < len; ++j) {
        crc ^= static_cast<uint8_t>(data[j]);
        for (int i = 0; i < 8; ++i) {
            if( (crc & 0x0001) > 0) {
                //0x1021 翻转  0x8408
                crc = (crc >> 1) ^ 0x8408;
            }
            else
                crc >>= 1;
        }
    }
    return crc;
}

void MotorSerialThread::readData()
{
    QByteArray data = serialPort->readAll();
    qDebug()<<data;
    qDebug()<<data.toHex();
}

void MotorSerialThread::run()
{
    QByteArray data;
    while(!stopped) {
        if(data.size() > 0)
            data.clear();
        if(serialStatus) {

//            SendAgreementContent(data, 0, 1, GetTuaSet(0.0), GetOmegaSet(-2*6.28*6.33), GetThetaSet(0.0), GetKpos(0.0), GetKspd(0.05));
            SendAgreementContent(data, 0, 1, GetTuaSet(0.0), GetOmegaSet(0.0), GetThetaSet(3.14*6.33), GetKpos(0.05), GetKspd(0.0));
            if(serialPort->waitForReadyRead(1)) {
                readData();
            }
            sleep(3000);
            if(data.size() > 0)
                data.clear();
            SendAgreementContent(data, 0, 1, GetTuaSet(0.0), GetOmegaSet(0.0), GetThetaSet(-3.14*6.33), GetKpos(0.05), GetKspd(0.0));
            if(serialPort->waitForReadyRead(1)) {
                readData();
            }
            sleep(3000);
        } else {
            stopped = true;
        }
    }
}

void MotorSerialThread::sleep(int msec)
{
    QTime dieTime = QTime::currentTime().addMSecs(msec);
    while( QTime::currentTime() < dieTime )
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}
