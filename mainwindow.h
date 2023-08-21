#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//#if _MSC_VER >= 1600
//#pragma execution_character_set("utf-8")
//#endif
#pragma execution_character_set("utf-8")
#include <QMessageBox>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QMainWindow>
#include <QFile>
#include <QList>
#include <QString>
#include <QTextStream>
#include <QDebug>
#include <QMessageBox>
#include <QUrl>
#include <QWebEnginePage>
#include <QWebChannel>
#include <QTimer>
#include <QFileDialog>
#include <QTime>
#include "mapchannel.h"
#include <QDateTime>
#include "QList"
#include <QThread>
#include <QtCore/QDateTime>
#include <QTcpServer>
#include <QTcpSocket>
#include <QMenu>
#include <QMenuBar>
#include <QTextStream>
#include <QFileDialog>
#include <QQueue>
#include "worker.h"
#include "myserial.h"
#include "qcustomplot.h"
//#include <QCPItemTracer>

#include "QDecContext.hh"
#include "QDecNumber.hh"
#include "QDecSingle.hh"
#include "QDecDouble.hh"
#include "QDecQuad.hh"
#include "QDecPacked.hh"

extern "C" {
#include "decimal64.h"
#include "decimal128.h"
#include "decPacked.h"
#include "decQuad.h"
}

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void update_gm(uint32_t pn,QString air, QString cnt);//更新底部标签数据
    void update_pack_cal(QVector<double> t);//更新一个数据包的计算
    void cal_ge();
    decNumber myLog2(decNumber input);
signals:
    void update(int);
    void writePort_sig(QByteArray data);
private slots:
    void updateTrack();
    void reloadMap();
    void setLocalIP(const QString &text);
    void on_psBt_listen_clicked();
    void handleNewConnectionSensor();
    void handleNewConnectionGPS();
    void handleDisconnected();
    void handleReadyRead();
    void HandleError(QSerialPort::SerialPortError error);
    void Update_uiHex(QByteArray &data);
    void tcpReadFrame();
    void myMoveMouseEvent(QMouseEvent *e);
private:
    Ui::MainWindow *ui;
    bool isLogSave=false;
    void mapWidgetInit();
    QWebChannel *channel;
    MapChannel *mapChannel;
    QTimer* timer0,*tcpOneShotTimer;
    double cur_jd=113.327789,cur_wd=23.090646;
    //QSerialPort mSerialPort;
    /**********************************************/
    QTcpServer *serverSensor=nullptr;
    QTcpServer *serverGPS=nullptr;
    QTcpSocket *Sensorclient=nullptr;
    QTcpSocket *GPSclient=nullptr;
    QQueue<QByteArray> queue;
    int sensorPort=0,GPSPort=0,write_cnt=0;
    QString gps_text="[23.00000,113.000000]";
    worker* w;//保存数据线程
    QSerialPort *m_port;//串口对象
    uint32_t rev_data_count=0;//测试
    QByteArray sensorData;//传感器数据输出
    QCustomPlot *customPlot;//曲线对象
    QLabel packNum,GmCNTRate,GmAirRate,imgSaveStatus;
    uint32_t packCnt=0;//接收数据包计数
    QList<QString> CAL_DATA;
    QString myE="0.1";//
    QString CPS_CNT="",AIR_RATE="";
    QCPItemTracer * plottracer;//曲线游标
};

#endif // MAINWINDOW_H
