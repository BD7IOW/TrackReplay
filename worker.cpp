#include "worker.h"
#include <QDebug>
worker::worker()
{
   wData.clear();
   isRun=true;
}
void worker::update_file()
{
   is_save_data=true;
}
void worker::worker_stop()
{
   isRun=false;
}
void worker::setwj(double jd,double wd)
{
   cur_jd=jd;
   cur_wd=wd;
}
void worker::run()
{

    while(isRun)
    {

        if(is_save_data)
        {
            //qDebug()<<"run....";
            is_save_data=false;
            QByteArray dataSave;
            dataSave.clear();
            while (!wData.isEmpty())
            {
                dataSave.append(wData.dequeue());
            }
            if(!dataSave.isEmpty())//准备保存数据
            {
                QFile file(QDir::currentPath()+"/Log.txt");
                file.open( QIODevice::ReadWrite | QIODevice::Text|QIODevice::Append );
                QTextStream out(&file);
                QDateTime currentDateTime = QDateTime::currentDateTime();
                // 将时间转换为字符串
                QString currentTimeString = currentDateTime.toString("yyyy-MM-dd hh:mm:ss");
                QString dataFrame="{\n\"time\":\""+currentTimeString+"\",\n";
                dataFrame+="\"loc\":\""+QString::number(cur_jd)+","+QString::number(cur_wd)+"\",\n";
                dataFrame+="\"data\":\""+dataSave.toHex(' ')+"\"\n}\n";
                out<< dataFrame;
                file.close();
            }
        }

        QThread::msleep(50);
    }
}
