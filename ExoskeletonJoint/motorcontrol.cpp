#include "motorcontrol.h"
#include <QDebug>
#include <math.h>
MotorControl::MotorControl()
{
    m_isStop = false;
    m_serial = new SerialPort("/dev/ttyUSB1");
    m_cmd = new MotorCmd;
    m_data = new MotorData;
    stopMode();
    m_lastPose = m_data->Pos;
}

MotorControl::~MotorControl()
{
    stopMode();
    m_isStop = true;
}

void MotorControl::stopMode()
{
    m_cmd->motorType = MotorType::GO_M8010_6;
    m_cmd->id    = 0;          //电机ID
    m_cmd->mode  = 0;          //0：停止；1：FOC；2：标定
    m_cmd->Pos   = 0.0;        //位置增量
    m_cmd->W     = 0.0;        //目标速度
    m_cmd->T     = 0.0;        //前馈力矩
    m_cmd->K_P   = 0.0;        //kp
    m_cmd->K_W   = 0.0;        //kd
    m_serial->sendRecv(m_cmd,m_data);
}

void MotorControl::run()
{
    int flag = 1;
    int num = 0;
    int number = 0;
    // if(m_lastPose > 0)
    //     flag = -1;
    m_cmd->K_P   = 0.2;        //kp
    m_cmd->K_W   = 0.01;        //kd
    while(!m_isStop) {
        m_cmd->motorType = MotorType::GO_M8010_6;
        m_cmd->id    = 0;          //电机ID
        m_cmd->mode  = 1;          //0：停止；1：FOC；2：标定
        m_cmd->Pos   = flag * 3.14*6.33;  //绝对位置
        m_cmd->W     = 0.0;        //目标速度
        m_cmd->T     = 0.0;        //前馈力矩
        // m_cmd->K_P   = 0.2;        //kp
        // m_cmd->K_W   = 0.01;        //kd
        m_serial->sendRecv(m_cmd, m_data);
        if(m_data->correct == true)
        {
            if(fabs(m_lastPose - m_data->Pos)<0.0005)
                num ++;
            else
                num = 0;
            // std::cout <<  std::endl;
            if(num < 4) {
                number++;
                std::cout <<"kp "<<m_cmd->K_P<<" kd "<<m_cmd->K_W<<" motor.Pos: "    << m_data->Pos    << " rad" << std::endl;
            } else {
                num = 0;
                if(fabs(fabs(fabs(m_data->Pos) - 19.8762) - 0.001) > 0.000001)
                // if(m_cmd->K_P < 0.37)
                    m_cmd->K_P += 0.001;
                else
                    m_isStop = true;
                std::cout <<"kp "<<m_cmd->K_P<<" kd "<<m_cmd->K_W<<" motor.Pos: "    << m_data->Pos    << " rad " <<number<< std::endl;
                flag *= -1;
                number = 0;
            }
            m_lastPose = m_data->Pos;
            // std::cout <<  "motor.Temp: "   << m_data->Temp   << " ℃"  << std::endl;
            // std::cout <<  "motor.W: "      << m_data->W      << " rad/s"<<std::endl;
            // std::cout <<  "motor.T: "      << m_data->T      << " N.m" << std::endl;
            // std::cout <<  "motor.MError: " << m_data->MError <<  std::endl;
            // std::cout <<  std::endl;
            // qDebug()<<"kp"<<m_cmd->K_P<<"kd"<<m_cmd->K_W<<"deltaPose"<<m_data->Pos - m_lastPose<<"rad "<<"motor.Pos:"<<m_data->Pos;
        }
        // 十六进制打印发送和接收指令
        // uint8_t *p = (uint8_t *)m_data->get_motor_recv_data();
        // for(int i =0; i<16; i++)
        //     printf("0X%02X ", *p++);
        // uint8_t *q = (uint8_t *)m_cmd->get_motor_send_data();
        // for(int i =0; i<17; i++)
        //     printf("0X%02X ", *q++);
        // for (int i = 0; i < 600; ++i) {
        //     for (int j = 0; j < 500; j++) {
        //         m_cmd->K_P += 0.001;
        //         m_cmd->K_W += 0.001;
        //     }
        // }
        usleep(200);
    }
}
