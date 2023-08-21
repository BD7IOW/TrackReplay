#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QHostInfo>
#include <QMetaType>
#include <cmath>




MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui -> setupUi(this);

    mapWidgetInit();

    ui->lineEdit_Host->setText("127.0.0.1");
    QString hostNme = QHostInfo::localHostName();
    QHostInfo hostInfo = QHostInfo::fromName(hostNme);
    auto addList = hostInfo.addresses();
    if(!addList.isEmpty())
    {
        ui->comboBox_IP->clear();
        ui->comboBox_IP->addItem("127.0.0.1");
        for (int i = 0;i < addList.count();++i)
        {
            QHostAddress hostAddress = addList.at(i);

            if (QAbstractSocket::IPv4Protocol==hostAddress.protocol())
            {
                auto ipAddr = hostAddress.toString();
                ui->comboBox_IP->addItem(ipAddr);
            }

        }

    }

    connect(ui->comboBox_IP, SIGNAL(currentTextChanged(QString)), this, SLOT(setLocalIP(QString)));

    ui->lineEdit_Port->setText("9003");
    ui->lineEdit_GPSPort->setText("9004");
    timer0 = new QTimer(this);
    connect(this->timer0,&QTimer::timeout,this,&MainWindow::updateTrack);

    ui->plainTextEdit_Sensor->setMaximumBlockCount(5000);
   // ui->plainTextEdit_GPS->setMaximumBlockCount(50000);//限制log窗口数据最大行数，防止数据太大系统崩溃
    timer0->start(10);
    tcpOneShotTimer = new QTimer(this);
    tcpOneShotTimer->setSingleShot(true);
    connect(this->tcpOneShotTimer,&QTimer::timeout,this,&MainWindow::tcpReadFrame);

    //tcpReadFrame
   // myPort = new SerialPort(this);
    //connect(this,SIGNAL(writePort_sig(QByteArray)),myPort,SLOT(write_data(QByteArray)));
    //connect(myPort,&SerialPort::serial_err_handle,this,&MainWindow::HandleError);
    connect(ui->pushButton_GetCOM,&QPushButton::clicked,this,[=]{
        QList<QSerialPortInfo> serialPortInfo = QSerialPortInfo::availablePorts();
            int count = serialPortInfo.count();
            for (int i=0;i<count;i++){
                ui->comboBox_COM->addItem(serialPortInfo.at(i).portName());   //循环将可用串口号添加到  选择下拉栏
            }
    });
    connect(ui->pushButton_OpenSerial,&QPushButton::clicked,this,[=]{
          if(m_port->isOpen())
          {
             m_port->close();

              ui->pushButton_OpenSerial->setText("打开串口");
          }else
          {
              QString mPortName = ui->comboBox_COM->currentText();
              quint32 baud = ui->comboBox_Baud->currentText().toUInt();
              m_port->setBaudRate(baud);
              m_port->setPortName(mPortName);
              m_port->setDataBits(QSerialPort::Data8);
              m_port->setParity(QSerialPort::NoParity);
              m_port->setStopBits(QSerialPort::OneStop);
              if(m_port->open(QIODevice::ReadWrite)) ui->pushButton_OpenSerial->setText("关闭串口");
              else
              {
                QMessageBox::warning(this, "失败", "串口打开失败");
                ui->pushButton_OpenSerial->setText("打开串口");
                m_port->close();
              }
          }
    });

    serverSensor = new QTcpServer(this);
    serverGPS = new QTcpServer(this);

    // 连接信号槽
    connect(serverSensor, &QTcpServer::newConnection, this, &MainWindow::handleNewConnectionSensor);
    connect(serverGPS, &QTcpServer::newConnection, this, &MainWindow::handleNewConnectionGPS);


    connect(ui->pushButton_Server,SIGNAL(clicked()),this,SLOT(on_psBt_listen_clicked()));
    connect(ui->checkBox_saveLog,&QCheckBox::stateChanged,this,[=](int status){
        if(status==Qt::CheckState::Checked)
        {
           isLogSave=true;

        }else
        {
           isLogSave=false;
        }
    });
    queue.clear();
    //save_quene.clear();
    w = new worker();
    w->start();
    //sw = new SerialWrite();
    //connect(sw,SIGNAL(sw->send_write_data(&)),this,SLOT(Update_uiHex(&)));
    //sw->setSerialPort(&mSerialPort);
    //sw->start();
    // 将串口和子类一同移入子线程
    //qRegisterMetaType<QQueue<QByteArray>>("queue");
    m_port = new QSerialPort(this);
    connect(m_port,&QSerialPort::errorOccurred,this,&MainWindow::HandleError);

    sensorData.clear();
    customPlot = new QCustomPlot(this);
    // 声明追踪变量是针对整个窗口
     plottracer = new QCPItemTracer(customPlot) ;
    //设置追踪曲线
    //设置基本坐标轴（左侧Y轴和下方X轴）可拖动、可缩放、曲线可选、legend可选、设置伸缩比例，使所有图例可见
    customPlot->setInteractions(QCP::iRangeDrag|QCP::iRangeZoom| QCP::iSelectAxes |
                                      QCP::iSelectLegend | QCP::iSelectPlottables);
    //customPlot->clearGraphs();
    customPlot->addGraph();
    QVector<double> x(5), y(5);
    for(uint8_t i=0;i<5;i++)
    {
        x.append(i);
        y.append(0);
    }
    customPlot->graph(0)->setData(y,x);
    plottracer->setGraph(customPlot->graph(0));
    //设置十字浮标样式
    QPen * pen = new QPen();
    pen->setColor(QColor(Qt::red));//黄色
    pen->setStyle(Qt::DashLine);//虚线
    plottracer->setPen(*pen);

    customPlot->xAxis->setRange(0, 4096);
    customPlot->xAxis->setLabel("Channel");             // 设置x轴的标签
    customPlot->yAxis->setLabel("Counts");
    ui->verticalLayout_5->addWidget(customPlot);
    connect(customPlot,SIGNAL(mouseMove(QMouseEvent *)),this,SLOT(myMoveMouseEvent(QMouseEvent *)));
    imgSaveStatus.setText("需要保存数据请勾选数据保存");
    update_gm(packCnt,QString("0"),QString("0"));
    // 将初始化的标签添加到底部状态栏上
    packNum.setMinimumWidth(300);
    GmAirRate.setMinimumWidth(600);
    GmCNTRate.setMinimumWidth(600);
    ui->statusBar->addWidget(&packNum);
    ui->statusBar->addWidget(&GmAirRate);
    ui->statusBar->addWidget(&GmCNTRate);
    ui->statusBar->addWidget(&imgSaveStatus);
    CAL_DATA.clear();//计算的数据，从软件目录的CAL.txt文件读取，格式E,A1,A2,A3
    //确保CAL数据只有一行，且全为英文状态下输入，打开软件初始化的时候取数据
    QFile file(QDir::currentPath()+"/CAL.txt");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QMessageBox::warning(this,"错误","读取CAL数据错误");
    }

    QTextStream stream(&file);
    QStringList row;
    while (!stream.atEnd())
    {
        QString line = stream.readLine();
        row = line.split(',', Qt::SkipEmptyParts);
    }
    for (uint8_t i=0;i<row.size();i++) {
        CAL_DATA.append(row.at(i).toLocal8Bit().constData());
    }
    ui->plainTextEdit_Sensor->appendPlainText("从程序目录读取CAL.txt文件数据:");
    if(CAL_DATA.size()>1)
    {

        ui->plainTextEdit_Sensor->appendPlainText("Emax="+CAL_DATA.at(0));
        for (int var = 0; var < (CAL_DATA.size()-1); ++var)
        {
         ui->plainTextEdit_Sensor->appendPlainText(QString("A%1=%2").arg(var).arg(CAL_DATA.at(1+var)));
        }
        cal_ge();
    }
    file.close();

    ui->plainTextEdit_Sensor->appendPlainText("若打开数据保存功能，接收到有效数据包后会把原始hex文件保存到程序目录下的log.txt文件，曲线图片则保存到img目录，每张图片的计算数据定位信息时间信息保存在img目录下的txt文件中。");


}
decNumber MainWindow::myLog2(decNumber input)
{
    decNumber temp2,output,Vlog2,VGE2;
    decContext set;
    decContextDefault(&set, DEC_INIT_BASE);
    set.traps=0;
    set.digits = 30;//小数精度
    set.emax = DEC_MAX_MATH;
    set.emin = -DEC_MAX_MATH;
    set.round=DEC_ROUND_HALF_EVEN;
    decNumberLog10(&output,&input,&set);
    decNumberFromUInt32(&temp2, 2);
    decNumberLog10(&Vlog2,&temp2,&set);
    decNumberDivide(&VGE2,&output,&Vlog2,&set);
    return VGE2;
}
void MainWindow::cal_ge()
{
  //参考https://speleotrove.com/decimal/dnnumb.html
  //参考https://github.com/semihc/qdecimal
  decNumber input,output,Const4096;
  decContext set; // working context
  decContextDefault(&set, DEC_INIT_BASE);
  set.traps=0;
  set.digits = 30;//小数精度
  set.emax = DEC_MAX_MATH;
  set.emin = -DEC_MAX_MATH;
  set.round=DEC_ROUND_HALF_EVEN;
  char string[300]={'\0'};

  decNumberFromString(&input, CAL_DATA.at(0).toUtf8().data(), &set);
  decNumberFromString(&Const4096, "4096", &set);
  decNumberDivide(&output,&input,&Const4096,&set);
  decNumberToString(&output,string);
  myE=QString::fromLocal8Bit(string,DECNUMDIGITS+14);
  ui->plainTextEdit_Sensor->appendPlainText("根据CAL文件数据计算(E/4096)="+myE+"\n");
}
void MainWindow::myMoveMouseEvent(QMouseEvent *e)
{
       if(customPlot->graph(0)==nullptr)return;
    //获取坐标,窗体鼠标的位置，不是曲线x轴的值
       int x_pos = e->pos().x();
   //    int y_pos = e->pos().y();
       //将鼠标坐标值换成曲线x轴的值
       float x_value = customPlot->xAxis->pixelToCoord(x_pos);
       // 获取x轴值对应的曲线中的y轴值
       float y_value = customPlot->graph(0)->data()->at(x_value)->value;
       //定义标签格式
       QString tip;
       tip = QString::number(x_value) + ",   " + QString::number(y_value);
       // 直接用tooltip显示
       QToolTip::showText(cursor().pos(),tip,customPlot);

       //按照x轴的值进行追踪
       plottracer->setGraphKey(x_value);
       //更新追踪位置
       plottracer->updatePosition();
       //更新曲线
       customPlot->replot();
}
void MainWindow::update_gm(uint32_t pn,QString air, QString cnt)
{
   packNum.setText("接收数据包数量:"+QString::number(pn));
   GmAirRate.setText("空气吸收剂量率(伽马): "+air+" uGy/S");
   GmCNTRate.setText("计数率(伽马): "+cnt+" CPS");
}
void MainWindow::update_pack_cal(QVector<double> t)
{//根据这4096个数据按公式计算
    //Q_UNUSED(t)
    decContext set; // working context
    decContextDefault(&set, DEC_INIT_BASE);
    set.traps=0;
    set.digits = 30;//小数精度
    set.emax = DEC_MAX_MATH;
    set.emin = -DEC_MAX_MATH;
    set.round=DEC_ROUND_HALF_EVEN;
    char string[300];
    char cntstring[300];
    decNumber CPS_IN,CPS_SUM,Const7118;
    decNumberFromString(&Const7118,"0.7118",&set);
    decNumberFromString(&CPS_SUM,"0",&set);
    uint32_t cps_sum=0;
    for (int i=0;i<t.size();i++)
    {
        cps_sum+=(uint32_t)t.at(i);
    }
    decNumberFromUInt32(&CPS_IN,cps_sum);
    decNumberDivide(&CPS_SUM,&CPS_IN,&Const7118,&set);//计数率
    decNumber Ee,Logres,index,BE,myPow,Ak,logComp,logpow,GE,Alog,CAlog;
    decNumber AirRate,CAirRate,MAirRate;
    //decNumberFromString(&Ee,"0",&set);//小e
    decNumberFromString(&Logres,"0",&set);
    decNumberFromString(&BE,myE.toUtf8().data(),&set);//BE=E/4096
    decNumberFromUInt32(&AirRate,0);
    decNumberFromUInt32(&CAirRate,0);
    uint32_t K = CAL_DATA.size()-1;//E,A0,A1,A2,A的个数
    for (uint16_t i=0;i< t.size();i++)
    {
        if((uint32_t)t.at(i)==0)continue;//等于0则跳过去
        decNumberFromUInt32(&index,i+1);//表示第几个数，从1开始
        decNumberFromUInt32(&CPS_IN,(uint32_t)t.at(i));//输入数据
        decNumberMultiply(&Ee,&index,&BE,&set);//E=数据编号*BE
        decNumberFromUInt32(&GE,0);//G(E)
        decNumberFromUInt32(&CAlog,0);//用于累加数据
        //decNumberToString(&Ee,string);
        //qDebug()<<"E="<<QString::fromUtf8(string,strlen(string));
        //logComp=myLog2(Ee);//计算log2(E)

        if(ui->comboBox_log->currentIndex()==2)decNumberLog10(&logComp,&Ee,&set);
        else if(ui->comboBox_log->currentIndex()==1)logComp=myLog2(Ee);
        else if(ui->comboBox_log->currentIndex()==0)
        {
          decNumberLn(&logComp,&Ee,&set);
        }
        //decNumberToString(&logComp,string);
        //qDebug()<<" "<<"log2(E)="<<QString::fromUtf8(string,strlen(string));

        for(uint16_t k=1;k<=K;k++)
        {//G(E)=0.1(log 1.953125)^2+0.2(log 1.953125)^1+0.3(log 1.953125)^0
             decNumberFromUInt32(&myPow,K-k);//幂
             decNumberFromString(&Ak,CAL_DATA.at(k).toUtf8().data(),&set);//Ak

             decNumberPower(&logpow,&logComp,&myPow,&set);
             decNumberMultiply(&Alog,&Ak,&logpow,&set);



             decNumberAdd(&CAlog,&GE,&Alog,&set);
             decNumberCopy(&GE,&CAlog);
             //decNumberToString(&GE,string);
             //qDebug()<<"A"<<k<<" GE="<<QString::fromUtf8(string,strlen(string));

        }
        decNumberMultiply(&MAirRate,&GE,&CPS_IN,&set);
        decNumberAdd(&CAirRate,&AirRate,&MAirRate,&set);
        decNumberCopy(&AirRate,&CAirRate);
        //qDebug()<<"数据编号:"<<(i+1);
        //decNumberToString(&GE,string);
        //qDebug()<<" "<<"data="<<(uint32_t)t.at(i)<<" GE="<<QString::fromUtf8(string,strlen(string));

    }

    packCnt++;
    memset((char*)string,'\0',sizeof (string));
    memset((char*)cntstring,'\0',sizeof (cntstring));
    decNumberDivide(&CAirRate,&AirRate,&Const7118,&set);
    decNumberToString(&CAirRate,string);
    decNumberToString(&CPS_SUM,cntstring);
    size_t ccnt=strlen(cntstring),aair=strlen(string);
    CPS_CNT=QString::fromUtf8(cntstring,ccnt);
    AIR_RATE=QString::fromUtf8(string,aair);
    update_gm(packCnt,AIR_RATE, CPS_CNT);

    if(ui->checkBox_saveLog->isChecked())
    {
       QString ts,path;
       QTime time = QTime::currentTime();
       ts=time.toString("[HH-mm-ss]");
       //path=gps_text+"-[CPS:"+CPS_CNT+",AIR:"+AIR_RATE+"]";

       path=QDir::currentPath()+"/img/"+QString("%1-%2.png").arg(ts).arg(packCnt);//ts
       QFile file(path);
       qDebug()<<path;
       if (!file.open( QIODevice::WriteOnly ))
       {
          imgSaveStatus.setText(QString("数据包%1图像保存失败").arg(packCnt));
       }else
       {
          imgSaveStatus.setText(QString("数据包%1图像保存成功").arg(packCnt));
          customPlot->savePng(path);
          QFile tfile(QDir::currentPath()+"/img/imgDec.txt");
          tfile.open( QIODevice::ReadWrite | QIODevice::Text|QIODevice::Append );
          QTextStream out(&tfile);
          QString dataTxt="\n{\n\"dec\":\""+QString("%1-%2.png").arg(ts).arg(packCnt)+"\","+"\n";
          dataTxt+="\"loc\":"+gps_text+",\n";
          dataTxt+="\"cps\":"+CPS_CNT+",\n";
          dataTxt+="\"airRate\":"+AIR_RATE+"\n}\n";
          out<<dataTxt;
          tfile.close();
       }
       file.close();
    }
}
void MainWindow::handleNewConnectionSensor()
{
    if(Sensorclient==nullptr)
    {
        Sensorclient = serverSensor->nextPendingConnection();

        connect(Sensorclient, &QTcpSocket::disconnected, this, &MainWindow::handleDisconnected);
        connect(Sensorclient, &QTcpSocket::readyRead, this, &MainWindow::handleReadyRead);
        ui->cmBx_client->addItem((tr("%1:%2")\
                               .arg(Sensorclient->peerAddress().toString())\
                               .arg(Sensorclient->peerPort())));
    }
}
void MainWindow::handleNewConnectionGPS()
{
    if(GPSclient==nullptr)
    {
        GPSclient = serverGPS->nextPendingConnection();
        connect(GPSclient, &QTcpSocket::disconnected, this, &MainWindow::handleDisconnected);
        connect(GPSclient, &QTcpSocket::readyRead, this, &MainWindow::handleReadyRead);
        ui->cmBx_client->addItem((tr("%1:%2")\
                               .arg(GPSclient->peerAddress().toString())\
                               .arg(GPSclient->peerPort())));
    }
}
void MainWindow::handleDisconnected()
{
    // 获取断开连接的socket
    QTcpSocket *socket = qobject_cast<QTcpSocket*>(sender());
    int p = socket->localPort();
    qDebug() << "Disconnected from" << socket->peerAddress().toString() << ":" << p;
    ui->cmBx_client->removeItem(ui->cmBx_client->findText(tr("%1:%2")\
                          .arg(socket->peerAddress().toString())\
                          .arg(socket->peerPort())));
    // 断开连接后删除socket对象

    socket->deleteLater();
    if(ui->lineEdit_Port->text().toInt()==p)
    {
        if(Sensorclient!=nullptr)
        {
            Sensorclient->deleteLater();
            Sensorclient=nullptr;
        }
    } else if(ui->lineEdit_GPSPort->text().toInt()==p)
    {
        if(GPSclient!=nullptr)
        {
            GPSclient->deleteLater();
            GPSclient=nullptr;
        }
    }
    rev_data_count=0;
    ui->lineEdit_Host->setDisabled(false);
    ui->lineEdit_GPSPort->setDisabled(false);
    ui->lineEdit_Port->setDisabled(false);
}
void MainWindow::handleReadyRead()
{
    // 获取有数据可读的socket
    QTcpSocket *socket = qobject_cast<QTcpSocket*>(sender());
   // int p = socket->peerPort();
    int lp = socket->localPort();
    //qDebug() << "Data received from" << socket->peerAddress().toString() << ":" << p<<"local:"<<lp;

    // 读取数据
    QByteArray data = socket->readAll();
    //qDebug()<<data.toHex(' ');
    if(lp==sensorPort)
    {
        tcpOneShotTimer->start(800);
        sensorData+=data;
        //queue.enqueue(data);
        //mySerialPort->_data.enqueue(data);
        //sw->send_quene.enqueue(data);//数据入FIFO
        //;
        rev_data_count+=data.size();
        //if(isLogSave)w->wData.enqueue(data);//用于数据保存，数据保存和GPS坐标关联，1s保存一次
    }else if(lp==GPSPort)
    {
        //[cur_wd,cur_jd]
        gps_text = QString(data);
    }
}
void MainWindow::tcpReadFrame()
{
    if(sensorData.size()!=8201)
    {
        sensorData.clear();
        qDebug()<<"sensor data len err";
        return;
    }
    uint32_t head=0;
    uint32_t temp=(uint8_t)sensorData.at(0);
    head|=(temp<<24);
    temp=(uint8_t)sensorData.at(1);
    head|=(temp<<16);
    temp=(uint8_t)sensorData.at(2);
    head|=(temp<<8);
    temp=(uint8_t)sensorData.at(3);
    head|=temp;
    if(head!=0xfffeeeee)
    {
        qDebug()<<"data head err";
        sensorData.clear();
        return;
    }
    head=0;
    temp=(uint8_t)sensorData.at(8197);
    head|=(temp<<24);
    temp=(uint8_t)sensorData.at(8198);
    head|=(temp<<16);
    temp=(uint8_t)sensorData.at(8199);
    head|=(temp<<8);
    temp=(uint8_t)sensorData.at(8200);
    head|=temp;
    if(head!=0xfffeffff)
    {
        qDebug()<<"data tail err";
        sensorData.clear();
        return;
    }
    QByteArray d = sensorData.mid(4,8192);
    uint8_t ck=0;
    for (uint16_t i=0;i<8192;i++) {
        ck+=(uint8_t)d.at(i);
    }
    if(ck!=(uint8_t)sensorData.at(8196))
    {
        qDebug()<<"checksum err";
        sensorData.clear();
        return;
    }
    QVector<double> x(4096), y(4096); // initialize with entries 0..100
    //QVector<double> yaix;
    for (int i=0; i<4096; ++i)
    {
      x[i] = i; // x goes from -1 to 1
      uint8_t *data_ptr =(uint8_t*)d.data();
      y[i] = ((uint16_t)data_ptr[i*2]<<8)|((uint16_t)data_ptr[i*2+1]); // let's plot a quadratic function
    }


    customPlot->graph(0)->setData(x, y);


    auto max = std::max_element(std::begin(y), std::end(y));
    customPlot->yAxis->setRange(0, (uint16_t)*max+20);
    customPlot->replot();
    update_pack_cal(y);

    queue.enqueue(d.mid(0,4096));
    queue.enqueue(d.mid(4096,4096));//传感器数据分开输出，一次输出串口助手会丢包
    if(isLogSave)w->wData.enqueue(d);
    sensorData.clear();
}
void MainWindow::on_psBt_listen_clicked()
{
    if(ui->pushButton_Server->text() == "Open") {
        QHostAddress addr;
        addr.setAddress(ui->lineEdit_Host->text());
        // 监听两个端口
        if (!serverGPS->listen(addr, ui->lineEdit_GPSPort->text().toUShort()) || !serverSensor->listen(addr, ui->lineEdit_Port->text().toUShort()))
        {
           ui->pushButton_Server->setText("Open");
           serverGPS->close();
           serverSensor->close();
           qDebug()<<"open server fail";
           return;
        }
         sensorPort=ui->lineEdit_Port->text().toInt();
         GPSPort = ui->lineEdit_GPSPort->text().toInt();
         ui->lineEdit_Host->setDisabled(true);
         ui->lineEdit_GPSPort->setDisabled(true);
         ui->lineEdit_Port->setDisabled(true);
         ui->pushButton_Server->setText("Close");
    }
    else {

            if(Sensorclient!=nullptr)
            {
              Sensorclient->disconnectFromHost();
             // Sensorclient->deleteLater();
              //Sensorclient=nullptr;
            }
            if(GPSclient!=nullptr)
            {
              GPSclient->disconnectFromHost();
              //GPSclient->deleteLater();
              //GPSclient=nullptr;
            }
            serverGPS->close();
            serverSensor->close();
            ui->cmBx_client->clear();
            ui->pushButton_Server->setText("Open");
        }



}



void MainWindow::setLocalIP(const QString &text)
{
     ui->lineEdit_Host->clear();
    ui->lineEdit_Host->setText(text);
}
void MainWindow::HandleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError)
    {
        QMessageBox::critical(this, "串口错误", m_port->errorString());
        ui->pushButton_OpenSerial->setText("打开串口");
        m_port->close();
     }
}


MainWindow::~MainWindow()
{
    //plottracer->pen().data_ptr();
    channel -> deregisterObject(mapChannel);
    w->worker_stop();
    w->quit();

    delete ui;
}

void MainWindow::mapWidgetInit()
{
    channel = new QWebChannel(this);
    mapChannel = new MapChannel(this);
    channel -> registerObject("passId",mapChannel);
    this -> ui -> widget_map -> page() -> setWebChannel(channel);
    this -> ui -> widget_map -> load(QUrl("file:///./map.html"));
    connect(mapChannel,&MapChannel::reloadMapClicked,this,&MainWindow::reloadMap);

    connect(mapChannel,&MapChannel::pointsCome,[](int index, double lng, double lat){
        qDebug()<<index<<QString::number(lng,'f',6)<<QString::number(lat,'f',6);
    });
}




void MainWindow::updateTrack()
{

   write_cnt++;
   if((write_cnt%100)==0)
   {
       //qDebug()<<rev_data_count;//打印tcp接收的实际数据量
       if(gps_text.at(0)=='[' && gps_text.at(gps_text.length() - 1)==']')
       {
           QString GPSdata = gps_text.mid(1, gps_text.length() - 2);
           ui->label_GPS->setText(GPSdata);
           QStringList list = GPSdata.split(",");
           cur_wd = list[0].toDouble();
           cur_jd = list[1].toDouble();
       }
       mapChannel->updateBoatPos(cur_jd,cur_wd,0);
       if(isLogSave)
       {
           w->setwj(cur_jd,cur_wd);
           w->update_file();
       }
   }
   if((write_cnt%5)==0)
   {
       if(!queue.isEmpty())
       {
           QByteArray dataRead = queue.dequeue();
           if(m_port->isOpen())m_port->write(dataRead);
           ui->plainTextEdit_Sensor->appendPlainText(dataRead.toHex(' '));
       }
   }

}

void MainWindow::Update_uiHex(QByteArray &data)
{
    ui->plainTextEdit_Sensor->appendPlainText(data.toHex(' '));
}

void MainWindow::reloadMap()
{
    this -> ui -> widget_map -> load(QUrl("file:///./onlinemap/map.html"));
}

