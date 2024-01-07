#ifndef SERIALPORTSENDTHREAD_H
#define SERIALPORTSENDTHREAD_H

#include <QThread>
#include <QObject>
#include <QSerialPort>
#include <QTimer>

class SerialPortSendThread : public QThread
{
public:
    explicit SerialPortSendThread(QObject *parent = nullptr);
    void sendData(const QByteArray& data);

protected:
    void run();

private:
    QSerialPort* m_serialPort;
    QByteArray m_data;
    QTimer m_timer;
};

#endif // SERIALPORTSENDTHREAD_H
