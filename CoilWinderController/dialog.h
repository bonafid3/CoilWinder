#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QUdpSocket>

#include <QDebug>

#define qd qDebug()

namespace Ui {
class Dialog;
}

struct sCommand {
    enum CmdType { cmdSTOP, cmdSTART };
    sCommand(CmdType t):type(t){}
    char pkt[3] = {'p', 'k', 't'};
    char type;
    uint32_t mainRPM=0;
    float wireDiameter=0;
    uint32_t coilLength=0;
    uint32_t direction=0;
};

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();

private slots:
    void on_bStart_clicked();
    void on_mUDPSocketReadyRead();
    void on_bStop_clicked();

private:
    Ui::Dialog *ui;
    QUdpSocket *mSock;

    QList<sCommand> mOut;

    int drawStartIdx=0;

    void sendCmd(sCommand &scmd);

    QByteArray readFile(const QString fname);
};

#endif // DIALOG_H
