#include "myserial.h"

MySerial::MySerial(QObject *parent) : QObject(parent)
{
    m_port = new QSerialPort();
    m_port->setDataBits(QSerialPort::Data8);
    m_port->setParity(QSerialPort::NoParity);
    m_port->setStopBits(QSerialPort::OneStop);
    connect(m_port,&QSerialPort::errorOccurred,this,
        [=](QSerialPort::SerialPortError error){
        //QSerialPort::SerialPortError error
        emit serialErr(error);
    });
    //_data.clear();
}
bool MySerial::serialOpen(QString COMx, quint32 baud)
{
    m_port->setBaudRate(baud);
    m_port->setPortName(COMx);
    //m_port->flush();
    return m_port->open(QIODevice::ReadWrite);
}

void MySerial::sendData(QByteArray data)
{
    if(m_port->isOpen())m_port->write(data);
}
void MySerial::serialClose()
{
   m_port->close();
}
QString MySerial::getSerialErrString()
{
    return m_port->errorString();
}
bool MySerial::serialIsOpen()
{
    return m_port->isOpen();
}
