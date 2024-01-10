#include "serialportmotor.h"
#include <math.h>
#include <QDebug>
SerialPortMotor::SerialPortMotor(QObject *parent)
    : QObject{parent}
{
    m_serialMotor = new QSerialPort;
    InitMotorPort();
    connect(m_serialMotor,SIGNAL(readyRead()),this,SLOT(ReceiveDataFromMotor()));
    connect(this,SIGNAL(sig_sendMotorData()),this,SLOT(SendDataToMotor()));
    // m_motorTimer = new QTimer(this);
    // connect(m_motorTimer,&QTimer::timeout,this,&SerialPortMotor::SendDataToMotor);
    // m_motorTimer->start(2);//ms
}

SerialPortMotor::~SerialPortMotor()
{
    if(m_serialMotor) {
        m_serialMotor->close();
        delete m_serialMotor;
    }
}

void SerialPortMotor::InitMotorPort()
{
    if(m_serialMotor->isOpen()) {
        m_serialMotor->close();
    }
    // Setup serial port configurations
    m_serialMotor->setPortName("/dev/ttyUSB1");
    m_serialMotor->setBaudRate(4000000);
    m_serialMotor->setDataBits(QSerialPort::Data8);     //串口数据8bit
    m_serialMotor->setParity(QSerialPort::NoParity);    //无奇偶校验位
    m_serialMotor->setStopBits(QSerialPort::OneStop);   //1bit停止位
    m_serialMotor->setFlowControl(QSerialPort::NoFlowControl);  //无流控制
    if (!m_serialMotor->open(QIODevice::ReadWrite)) {
        m_serialMotorStatus = false;
        qDebug("MotorSerialport opening failed!");
    } else {
        m_serialMotorStatus = true;
        qDebug("MotorSerialport opened successfully!");
        QByteArray data;
        SendAgreementContent(data, 0, 0, GetTuaSet(0.0), GetOmegaSet(6.28*6.33), GetThetaSet(0), GetKpos(0), GetKspd(0.05));
    }
}

void SerialPortMotor::SendDataToMotor()
{
    QByteArray data;
    SendAgreementContent(data, 0, 1, GetTuaSet(0.0), GetOmegaSet(6.28*6.33), GetThetaSet(0), GetKpos(0), GetKspd(0.05));
}

void SerialPortMotor::ReceiveDataFromMotor()
{
    QByteArray data = m_serialMotor->readAll();
    // qDebug()<<data;
    qDebug()<<"motordata"<<data.toHex();
    emit sig_sendMotorData();
}

void SerialPortMotor::SendAgreementContent(QByteArray &serialdata, uint8_t motorid, uint8_t status, int16_t tua, int16_t omega, int32_t theta, int16_t kpos, int16_t kspd)
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
    // qDebug()<<"motorsenddata"<<serialdata.toHex();
    //发送
    m_serialMotor->write(serialdata);
    m_serialMotor->flush();
}
int16_t SerialPortMotor::GetTuaSet(float tua)
{
    if(fabs(tua) <= 127.99) {
        return (int16_t)(tua * 256);
    } else {
        return static_cast<int16_t>(0.0);
    }
}

int16_t SerialPortMotor::GetOmegaSet(float omega)
{
    if(fabs(omega) <= 804.0) {
        return static_cast<int16_t>(omega * 256 / 2 / M_PI);
    } else {
        return static_cast<int16_t>(0.0);
    }
}

int32_t SerialPortMotor::GetThetaSet(float theta)
{
    if(fabs(theta) <= 411774.0) {
        return static_cast<int32_t>(theta * 32768 / 2 / M_PI);
    } else {
        return static_cast<int32_t>(0.0);
    }
}

int16_t SerialPortMotor::GetKpos(float kp)
{
    if(kp >= 0 && kp <= 25.599) {
        return static_cast<int16_t>(kp * 1280);
    } else {
        return static_cast<int16_t>(0.0);
    }
}

int16_t SerialPortMotor::GetKspd(float kw)
{
    if(kw >= 0 && kw <= 25.599) {
        return static_cast<int16_t>(kw * 1280);
    } else {
        return static_cast<int16_t>(0.0);
    }
}

uint16_t SerialPortMotor::CRC16CCITT(const QByteArray &data)
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

void SerialPortMotor::parseMotorData(QByteArray &data)
{

}
