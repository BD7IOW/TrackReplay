#ifndef MYSERIAL_H
#define MYSERIAL_H
#include <QSerialPort>
#include <QObject>
#include <QQueue>
class MySerial : public QObject
{
    Q_OBJECT
public:
    explicit MySerial(QObject *parent = nullptr);
    bool serialOpen(QString COMx,quint32 baud);
    bool serialIsOpen();
    void serialClose();
    QString getSerialErrString();
    //QQueue<QByteArray> _data;
signals:
   void serialErr(QSerialPort::SerialPortError error);
public slots:
    void sendData(QByteArray data);

private:
    QSerialPort *m_port;
};

#endif // MYSERIAL_H
