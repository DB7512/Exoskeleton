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
        SendAgreementContent(data, 0, 0, GetTuaSet(0.0), GetOmegaSet(-6.28*6.33), GetThetaSet(0), GetKpos(0), GetKspd(0.05));
    }
}

void SerialPortMotor::SendDataToMotor()
{
    QByteArray data;
    SendAgreementContent(data, 0, 0, GetTuaSet(0.0), GetOmegaSet(-6.28*6.33), GetThetaSet(0), GetKpos(0), GetKspd(0.05));
}

void SerialPortMotor::ReceiveDataFromMotor()
{
    QByteArray data = m_serialMotor->readAll();
    // qDebug()<<"motorreceiveddata"<<data.toHex();
    parseMotorData(data);
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
    // 十六进制数据转换成十进制数据时需要注意是否带符号！！！
    if(!data.isEmpty() && data.size() == 16) {
        bool ok;
        // qDebug()<<"datasize"<<data.size();
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
        // qDebug()<<"motordata"<<motorData.toHex()<<"size"<<motorData.size();
        //实际关节输出转矩2bytes
        QByteArray motorTua;
        motorTua.append(motorData[1]);
        motorTua.append(motorData[0]);
        QString tuaStr = motorTua.toHex();
        int16_t tuaValue = tuaStr.toInt(&ok,16);
        float tua = tuaValue / 256.0;

        //实际关节输出速度2bytes
        QByteArray motorOmega;
        motorOmega.append(motorData[3]);
        motorOmega.append(motorData[2]);
        QString omegaStr = motorOmega.toHex();
        int16_t omegaValue = omegaStr.toInt(&ok,16);
        float omega = omegaValue / 256.0 * 2 * M_PI;

        //实际关节输出位置（多圈累加）4bytes
        QByteArray motorTheta;
        motorTheta.append(motorData[7]);
        motorTheta.append(motorData[6]);
        motorTheta.append(motorData[5]);
        motorTheta.append(motorData[4]);
        QString thetaStr = motorTheta.toHex();
        int32_t thetaValue = thetaStr.toLong(&ok,16);
        float theta = thetaValue / 32768.0 * 2 * M_PI;

        //电机温度1byte
        int8_t temp = static_cast<int8_t>(motorData[8]);
        //电机错误标识3bits，足端力12bits
        QByteArray motorInformation;
        motorInformation.append(motorData[10]);
        motorInformation.append(motorData[9]);
        QString informationStr = motorInformation.toHex();
        uint16_t informationValue = informationStr.toUInt(&ok,16);
        // 电机错误标识，0：正常，1：过热，2：过流，3：过压，4：编码器故障
        uint8_t motorError = informationValue & 0x07;
        // 足端力
        uint16_t motorForce = (informationValue >> 3) & 0x0FFF;

        qDebug()<<"id"<<motorId<<"status"<<motorStatus<<"tua"<<tua
                 <<"omega"<<omega<<"theta"<<theta<<"temp"<<temp
                 <<"error"<<motorError<<"force"<<motorForce;
    } else {
        qDebug("Incomplete received data!!!");
    }
}
