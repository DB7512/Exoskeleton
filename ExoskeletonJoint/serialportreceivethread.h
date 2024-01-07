#ifndef SERIALPORTRECEIVETHREAD_H
#define SERIALPORTRECEIVETHREAD_H

#include <QThread>
#include <QObject>
#include <QSerialPort>

class SerialPortReceiveThread : public QThread
{
public:
    explicit SerialPortReceiveThread(QObject *parent = nullptr);

signals:
    void dataReceived(const QByteArray& data);

protected:
    void run();

private:
    QSerialPort* m_serialPort;
};

#endif // SERIALPORTRECEIVETHREAD_H
