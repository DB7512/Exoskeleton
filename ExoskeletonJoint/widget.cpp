#include "widget.h"
#include "ui_widget.h"
#include "serialportmanager.h"

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    //开启机器人控制线程
    QThread *robotThread = new QThread();
    m_robotControl = new RobotControl;
    m_robotControl->moveToThread(robotThread);
    connect(robotThread,SIGNAL(started()),m_robotControl,SLOT(startThreadSlot()));
    robotThread->start();
}

Widget::~Widget()
{
    delete ui;
}

void Widget::on_pushButton_clicked()
{

}

