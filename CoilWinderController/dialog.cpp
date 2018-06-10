#include "dialog.h"
#include "ui_dialog.h"

#include <QFile>

static QHostAddress addr("192.168.0.255");

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);

    setMouseTracking(true);

    mSock = new QUdpSocket(this);
    mSock->bind(QHostAddress::Any, 8080);

    connect(mSock, SIGNAL(readyRead()), this, SLOT(on_mUDPSocketReadyRead()));

    update();
}

QByteArray Dialog::readFile(const QString fname)
{
    QByteArray res;
    QFile f(fname);
    if(f.open(QFile::ReadOnly)){
        res = f.readAll();
        f.close();
    }
    return res;
}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::on_mUDPSocketReadyRead()
{
    qd << "udp socket ready read";

    QUdpSocket *sock = static_cast<QUdpSocket*>(sender());

    if(sock) {
        while (sock->hasPendingDatagrams())
        {
            QByteArray datagram;
            datagram.resize(sock->pendingDatagramSize());
            QHostAddress sender;
            quint16 senderPort;

            sock->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

            addr = sender;

            datagram = datagram.trimmed();

            qd << "got datagram:"<<datagram;

            if(mOut.size()) {
                sendCmd(mOut.front()); mOut.pop_front();
            }
        }
    }
}

void Dialog::sendCmd(sCommand& scmd)
{
    qd << "sCommand size:" << sizeof(sCommand);

    QByteArray data;
    data.resize(sizeof(sCommand));
    memcpy(data.data(), (void*)&scmd, sizeof(sCommand));

    qd << "sent" << mSock->writeDatagram(data, addr, 8080) << "bytes";
}

void Dialog::on_bStart_clicked()
{
    sCommand cmd(sCommand::CmdType::cmdSTART);
    cmd.mainRPM = ui->sb1stMotorSpeed->value();
    cmd.wireDiameter = ui->sbWireDiameter->value();
    cmd.coilLength = ui->sbCoilLength->value();
    cmd.direction = ui->sbDirection->value();

    sendCmd(cmd);
}

void Dialog::on_bStop_clicked()
{
    sCommand cmd(sCommand::CmdType::cmdSTOP);
    sendCmd(cmd);
}
