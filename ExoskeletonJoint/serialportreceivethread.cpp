#include "serialportreceivethread.h"

SerialPortReceiveThread::SerialPortReceiveThread(QObject *parent) : QThread(parent)
{

}

void SerialPortReceiveThread::run()
{
    while (!isInterruptionRequested()) {
        if(m_serialPort->waitForReadyRead(100)) {
            QByteArray data = m_serialPort->readAll();
            emit dataReceived(data);
        }
    }
}
