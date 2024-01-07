#include "serialportsendthread.h"

SerialPortSendThread::SerialPortSendThread(QObject *parent) : QThread(parent)
{
    connect(&m_timer, &QTimer::timeout, this, &SerialPortSendThread::sendData);
}

void SerialPortSendThread::sendData(const QByteArray &data)
{
    m_data = data;
}

void SerialPortSendThread::run()
{
    while (!isInterruptionRequested()) {
        if (!m_data.isEmpty()) {
            m_serialPort->write(m_data);
            m_serialPort->waitForBytesWritten(100);
            m_data.clear();
        }
    }
}
