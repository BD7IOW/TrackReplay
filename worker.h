#ifndef WORKER_H
#define WORKER_H

#include <QThread>
#include <QObject>
#include <QQueue>
#include <QFile>
#include <QDateTime>
#include <QUrl>
#include <QFileDialog>
#include <QTextStream>


class worker : public QThread
{
    Q_OBJECT
public:
    explicit worker();
    void run() override;
public :
    QQueue<QByteArray> wData;
    double cur_jd=0,cur_wd=0;
    bool is_save_data=false,isRun=true;
    void update_file();
    void worker_stop();
    void setwj(double jd,double wd);
};

#endif // WORKER_H
