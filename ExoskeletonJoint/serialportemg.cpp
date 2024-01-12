#include "serialportemg.h"
#include <QDebug>
#include <vector>
SerialPortEMG::SerialPortEMG(QObject *parent)
    : QObject{parent}
{
    m_serialEMG = new QSerialPort;
    InitEMGPort();
    connect(m_serialEMG,SIGNAL(readyRead()),this,SLOT(ReceiveEMGData()));
    m_EMGData.clear();
}

SerialPortEMG::~SerialPortEMG()
{
    if(m_serialEMG) {
        if(m_serialEMG->isOpen())
            m_serialEMG->close();
        delete m_serialEMG;
    }
}

void SerialPortEMG::InitEMGPort()
{
    if(m_serialEMG->isOpen()) {
        m_serialEMG->close();
    }
    // Setup serial port configurations
    m_serialEMG->setPortName("/dev/ttyUSB2");
    m_serialEMG->setBaudRate(115200);
    m_serialEMG->setDataBits(QSerialPort::Data8);     //串口数据8bit
    m_serialEMG->setParity(QSerialPort::NoParity);    //无奇偶校验位
    m_serialEMG->setStopBits(QSerialPort::OneStop);   //1bit停止位
    m_serialEMG->setFlowControl(QSerialPort::NoFlowControl);  //无流控制
    if (!m_serialEMG->open(QIODevice::ReadWrite)) {
        m_serialEMGStatus = false;
        qDebug("EMGSerialport opening failed!");
    } else {
        m_serialEMGStatus = true;
        qDebug("EMGSerialport opened successfully!");
    }
}

void SerialPortEMG::ReceiveEMGData()
{
    // EMG串口以ASCII码读取数据，格式：data1,data2,data3,data4,data5,data6\r\n
    // 存在“data1,data2,data3,data4,data5,data6\r\ndata1,data2,data3,”这种数据不完整情况，需要处理；

    QByteArray data = m_serialEMG->readAll();
    QString dataStr = data;
    parseEMGData(m_EMGData);
    qDebug()<<"EMGdata"<<dataStr;
}

void SerialPortEMG::parseEMGData(QString &data)
{
    // 未测试
    m_EMGData.append(data);
    // data = "111,121,1341,123,1245,12367\r\n1234,13,";
    vector<int> EMGData(6);
    QStringList list = m_EMGData.split("\r\n");
    QString tempStr;
    if (list.size() > 0) {
        for(int i = 0; i < list.size(); ++i) {
            if(i < list.size() - 1) {
                tempStr = list[i];
                QStringList listData = tempStr.split(",");
                for(int j = 0; j < listData.size(); ++j) {
                    tempStr = listData[j];
                    EMGData[j] = tempStr.toInt();
                }
                m_EMGMessage.push_back(EMGData);
                // 获得的emg数据暂未处理，后续考虑使用3个寄存器保存数据供处理
                if (m_EMGMessage.size() > 1000)
                    m_EMGMessage.clear();
            } else {
                m_EMGData.clear();
                m_EMGData.append(list[i]);
                break;
            }
        }
    }
}
