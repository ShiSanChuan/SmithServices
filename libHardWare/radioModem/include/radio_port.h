////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      RadioPort Code for Robot
///ALL RIGHTS RESERVED
///@file:serial_port.h
///@brief: ubuntu 下通用串口模块头文件
///实现串口通信。参考RM2017@uestc代码
///@vesion 1.0
///@author: Gezp
///@email: 1350824033@qq.com
///@date: 2017.12.8
///修订历史：
/// 12.19: 参考2017RM及网上开源代码
///2018.3.9:更改数据帧协议，哨兵为16Byte数据帧。
///2018.4.28：清理程序，串口通用模块，不再包含协议部分。
///2018.4.29:增加重连函数，通过遍端口1-10实现连接串口。
///2018.5.17：取消重连函数，保留重连方法，在收发数据时，异常时会自动重连。
/// （udev固定设备路径，串口发送失败，重连即可）
////////////////////////////////////////////////////////////////////////////////

#ifndef RADIO_PORT_H
#define RADIO_PORT_H

#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <unistd.h>     /*Unix 标准函数定义*/
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX 终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <string.h>
#include <iostream>

//暂不理解以下含义
#define RESET -1
#define SET 0


typedef enum error
{
    e_true_t = 0,
    e_openfile_t,
    e_fcntl_t,
    e_terminal_t,
    e_openseril_t,
    e_datasize_t,
    e_parity_t,
    e_stopbits_t,
    e_activate_t,
    e_seterr_t

}Serial_error_t;




class RadioPort
{
public:
    RadioPort(void);
    ~RadioPort(void);

private:
    bool init();
    int Param_Set(int m_speed, int m_flow_ctrl, int m_databits, int m_stopbits, int m_parity);
    bool Flush(void);
public:
    bool init(std::string port,int speed=115200,int flowCtrl=0,int databits=8,int stopbits=1,int m_parity='N');
    int Recv(unsigned char *rcv_buf,int data_len);
    int Send(unsigned char *send_buf,int data_len);
    void ShowParam(void);
    bool isOpen();

private:
    Serial_error_t err;
    std::string mPort;   //端口设备名
    int mSpeed;
    int mFlowCtrl;
    int fd;
    int mDatabits;
    int mStopbits;
    int mParity;
private:
    bool mIsOpen;
};










#endif
