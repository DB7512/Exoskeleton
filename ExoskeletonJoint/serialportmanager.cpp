#include "serialportmanager.h"
#include <QThread>
#include <QDebug>
#include <math.h>
#include "serialportreceivethread.h"
#include "serialportsendthread.h"

SerialPortManager::SerialPortManager(QObject *parent) : QObject(parent)
{
    // 初始化串口设置
    setupPort(m_serialPortMotor, "/dev/ttyUSB0", 4000000);
    setupPort(m_serialPortEncoder, "/dev/ttyUSB1", 115200);

    if (m_serialPortEncoder.open(QIODevice::ReadWrite) && m_serialPortMotor.open(QIODevice::ReadWrite)) {
        // 创建并启动接收线程和发送线程
        SerialPortReceiveThread receiveThreadMotor(&m_serialPortMotor);
        SerialPortSendThread sendThreadMotor(&m_serialPortMotor);

        SerialPortReceiveThread receiveThreadEncoder(&m_serialPortEncoder);
        SerialPortSendThread sendThreadEncoder(&m_serialPortEncoder);

        receiveThreadMotor.start();
        sendThreadMotor.start();

        receiveThreadEncoder.start();
        sendThreadEncoder.start();

        connect(&receiveThreadMotor, &SerialPortReceiveThread::dataReceived, this, &SerialPortManager::handleMotorPortDataReceived);
        connect(&receiveThreadEncoder, &SerialPortReceiveThread::dataReceived, this, &SerialPortManager::handleEncoderPortDataReceived);

//        m_timerMotor = new QTimer;
//        m_timerMotor->setInterval(2); // 500Hz = 1ms周期，但考虑其他处理时间，设置为2ms
//        connect(m_timerMotor, &QTimer::timeout, this, &SerialPortManager::sendMotorPortData);

//        m_timerEncoder = new QTimer;
//        m_timerEncoder->setInterval(2); // 同样设置为2ms
//        connect(m_timerEncoder, &QTimer::timeout, this, &SerialPortManager::sendEncoderPortData);
    }
}

SerialPortManager::~SerialPortManager()
{

}

void SerialPortManager::start()
{
    if (m_serialPortMotor.open(QIODevice::ReadWrite) && m_serialPortEncoder.open(QIODevice::ReadWrite)) {
        qDebug("Serialport opened successfully!");
        m_timerMotor->start();
        m_timerEncoder->start();
    }
}

void SerialPortManager::handleMotorPortDataReceived(const QByteArray &data)
{
//    QByteArray data = m_serialPortEncoder.readAll();
    qDebug()<<data;
    qDebug()<<data.toHex();
    if(!data.isEmpty()) {
        /*数据16bytes，小端模式*/
        //模式信息1byte
        uint8_t motorMode = data[2];
        uint8_t motorId = 0;
        uint8_t motorStatus = 0;
        motorId |= motorMode & 0x0F;
        motorStatus |= (motorMode >> 4) & 0x07;
        /*反馈数据11bytes*/
        QByteArray motorData;
        for(int i = 0; i < 11; ++i)
            motorData.append(data[3 + i]);
        //实际关节输出转矩2bytes
        QByteArray motorTua;
        motorTua.append(motorData[0]);
        motorTua.append(motorData[1]);
        int16_t  tuaValue = 0;
        for(int i = 0; i < 2; ++i) {
            int16_t tuaByte = static_cast<int8_t>(motorTua[i]);
            tuaValue |= tuaByte << (8 * (1 - i));
        }
        float tua = tuaValue / 256.0;

        //实际关节输出速度2bytes
        QByteArray motorOmega;
        motorOmega.append(motorData[2]);
        motorOmega.append(motorData[3]);
        int16_t  omegaValue = 0;
        for(int i = 0; i < 2; ++i) {
            int16_t omegaByte = static_cast<int8_t>(motorOmega[i]);
            omegaValue |= omegaByte << (8 * (1 - i));
        }
        float omega = omegaValue / 256.0 * 2 * M_PI;

        //实际关节输出位置（多圈累加）4bytes
        QByteArray motorTheta;
        motorTheta.append(motorData[4]);
        motorTheta.append(motorData[5]);
        motorTheta.append(motorData[6]);
        motorTheta.append(motorData[7]);
        int32_t thetaValue = 0;
        for(int i = 0; i < 4; ++i) {
            int32_t thetaByte = static_cast<int8_t>(motorOmega[i]);
            thetaValue |= thetaByte << (8 * (3 - i));
        }
        float theta = thetaValue / 32768.0 * 2 * M_PI;

        //电机温度1byte
        int8_t temp = static_cast<int8_t>(motorData[8]);
        //电机错误标识3bits，足端力12bits
        QByteArray motorInformation;
        motorInformation.append(motorData[9]);
        motorInformation.append(motorData[10]);
        int32_t informationValue = 0;
        for(int i = 0; i < 2; ++i) {
            int16_t informationByte = static_cast<int8_t>(motorOmega[i]);
            informationValue |= informationByte << (8 * (1 - i));
        }
        uint8_t motorError = informationValue & 0x07;
        int16_t motorForce = (informationValue >> 3) & 0x0FFF;
        qDebug()<<"id"<<motorId<<"status"<<motorStatus<<"tua"<<tua
               <<"omega"<<omega<<"theta"<<theta<<"temp"<<temp
              <<"error"<<motorError<<"force"<<motorForce;
    }
}

void SerialPortManager::handleEncoderPortDataReceived(const QByteArray& data)
{
//    QByteArray data = m_serialPortEncoder.readAll();
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

void SerialPortManager::sendMotorPortData()
{
    QByteArray data;
    SendAgreementContent(data, 0, 1, GetTuaSet(0.0), GetOmegaSet(-2*6.28*6.33), GetThetaSet(0.0), GetKpos(0.0), GetKspd(0.05));
    //发送
    m_serialPortMotor.write(data);
    m_serialPortMotor.flush();
}

void SerialPortManager::sendEncoderPortData()
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
    m_serialPortEncoder.write(data);
    m_serialPortEncoder.flush();
}

void SerialPortManager::startThreadSlot()
{

}

void SerialPortManager::setupPort(QSerialPort &port, const QString &portName, int baudRate)
{
    // Setup serial port configurations
    port.setPortName(portName);
    port.setBaudRate(baudRate);
    port.setDataBits(QSerialPort::Data8);     //串口数据8bit
    port.setParity(QSerialPort::NoParity);    //无奇偶校验位
    port.setStopBits(QSerialPort::OneStop);   //1bit停止位
    port.setFlowControl(QSerialPort::NoFlowControl);  //无流控制
}

void SerialPortManager::SendAgreementContent(QByteArray &serialdata, uint8_t motorid, uint8_t status, int16_t tua, int16_t omega, int32_t theta, int16_t kpos, int16_t kspd)
{
    if(serialdata.length() > 0)
        serialdata.clear();

    serialdata.append(0xFE);
    serialdata.append(0xEE);

    /*小端模式：低字节在低地址，高字节在高地址，cpu读取内存从低地址开始读取,数据强制转换时不用对字节进行调整*/
    //模式设置 1byte
    int8_t lowNibble = static_cast<uint8_t>(motorid) & 0x0F;
    int8_t highNibble = static_cast<int8_t>(status) << 4;
    int8_t mode = highNibble | lowNibble;
    serialdata.append(mode);

    /*控制参数*/
    //tua 2bytes
    int8_t highByteTua = static_cast<int8_t>((tua >> 8) & 0xFF);
    int8_t lowByteTua = static_cast<int8_t>(tua & 0xFF);
    serialdata.append(highByteTua);
    serialdata.append(lowByteTua);

    //omega 2bytes
    int8_t highByteOmega = static_cast<int8_t>((omega >> 8) & 0xFF);
    int8_t lowByteOmega = static_cast<int8_t>(omega & 0xFF);
    serialdata.append(highByteOmega);
    serialdata.append(lowByteOmega);

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
    serialdata.append(highByteKspd);
    serialdata.append(lowByteKspd);

    //crc 2bytes
    int len = serialdata.size();
    uint16_t crc = CRC16CCITT(serialdata, len);
    int8_t highByteCRC = static_cast<int8_t>((crc >> 8) & 0xFF);
    int8_t lowByteCRC = static_cast<int8_t>(crc & 0xFF);
    serialdata.append(lowByteCRC);
    serialdata.append(highByteCRC);
    //    qDebug()<<"motorsenddata"<<serialdata.toHex();
}

int16_t SerialPortManager::GetTuaSet(float tua)
{
    if(fabs(tua) <= 127.99)
        return (int16_t)(tua * 256);
    else
        return static_cast<int16_t>(0.0);
}

int16_t SerialPortManager::GetOmegaSet(float omega)
{
    if(fabs(omega) <= 804.0)
        return static_cast<int16_t>(omega * 256 / 2 / M_PI);
    else
        return static_cast<int16_t>(0.0);
}

int32_t SerialPortManager::GetThetaSet(float theta)
{
    if(fabs(theta) <= 411774.0)
        return static_cast<int32_t>(theta * 32768 / 2 / M_PI);
    else
        return static_cast<int32_t>(0.0);
}

int16_t SerialPortManager::GetKpos(float kp)
{
    if(kp >= 0 && kp <= 25.599)
        return static_cast<int16_t>(kp * 1280);
    else
        return static_cast<int16_t>(0.0);
}

int16_t SerialPortManager::GetKspd(float kw)
{
    if(kw >= 0 && kw <= 25.599)
        return static_cast<int16_t>(kw * 1280);
    else
        return static_cast<int16_t>(0.0);
}

uint16_t SerialPortManager::CRC16CCITT(const QByteArray &data, int len)
{
    // CRC16_CCITT 计算方法
    //初始值
    uint16_t crc = 0x0000;
    //int len = data.size();
    for(int j = 0; j < len; ++j) {
        crc ^= static_cast<uint8_t>(data[j]);
        for (int i = 0; i < 8; ++i) {
            if( (crc & 0x0001) > 0)
                //0x1021 翻转  0x8408
                crc = (crc >> 1) ^ 0x8408;
            else
                crc >>= 1;
        }
    }
    return crc;
}
