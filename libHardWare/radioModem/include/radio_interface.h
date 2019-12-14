////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      SerialPort Code for robot
///ALL RIGHTS RESERVED
///@file:serial_interface.h
///@brief: 机器人控制基本接口头文件，包含对车底盘及云台的基本接口。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-3-4
///修订历史：
///2018.6.13:哨兵协议versio2.1
////////////////////////////////////////////////////////////////////////////////
#include "radio_port.h"
#include "radio_packet.h"

#ifndef RMDEMO_RADIO_INTERFACE_H
#define RMDEMO_RADIO_INTERFACE_H

typedef enum:unsigned char {
    CMD_PID_UPDATE = 0xd0,
    CMD_PID_SAVE = 0xd1,
    CMD_REGET_FLIGHT_CONTROL = 0xd2,

    CMD_IN_INIT = 0x00,//初始化启动
    CMD_IN_RETURN = 0x10,//返回
    CMD_IN_MOVE = 0x20,//移动
    CMD_IN_STAB = 0x40,//刺气球
    CMD_IN_CATCH = 0x80,//抓气球    
} RadioPortCMD;


class RadioInterface
{
public:
    RadioInterface(void);
    ~RadioInterface(void);

private:
    RadioPort mMcuSerialPort;

public:

    struct Point2f{
        float pitch;
        float yaw;
        Point2f(){}
        Point2f(float p, float y):pitch(p), yaw(y){}
    };

    /** 初始化函数
    *  @param:  std::string devPath :串口设备路径
    *  @return: int :错误号，0代表无错误，１代表发生错误。
    */
    int init(std::string devPath);

    //查询串口是否打开
    bool isOpen();

    /** 命令数据发送函数
     *根据协议数据帧格式，封装串口发送函数。
     *  @param:  RadioPacket sendPacket :待发送的数据包
     *  @return: int :错误号，0代表无错误，１代表发生错误。
     *  @note:   数据包构成参考serial_packet.h/cpp
     */
    int dataSend(RadioPacket sendPacket);

    /** 命令数据接收函数
     *根据协议数据帧格式，封装串口接收函数，包括协议帧的检查机制。
     *  @return: int :错误号，0代表无错误，１代表发生错误。
     *  @note:   数据包构成参考serial_packet.h/cpp
     */
    int dataRecv(RadioPacket &recvPacket);
        /** 命令数据发送函数
     *根据协议数据帧格式，封装串口发送函数。
     *  @param:  RadioPacket sendPacket :待发送的数据包
     *  @return: int :错误号，0代表无错误，１代表发生错误。
     *  @note:   数据包构成参考serial_packet.h/cpp
     */
    int dataSend32(RadioPacket sendPacket);

    /** 命令数据接收函数
     *根据协议数据帧格式，封装串口接收函数，包括协议帧的检查机制。
     *  @return: int :错误号，0代表无错误，１代表发生错误。
     *  @note:   数据包构成参考serial_packet.h/cpp
     */
    int dataRecv32(RadioPacket &recvPacket);

    /** 底盘停止设置
    *  @param:  void
    *  @return: void
    *  @note :
    */
    void chassisStopSet();



    /** 车云台pitch轴,yaw轴角度增量设置函数（增量形式，相对坐标系）
    *  @param:  short int pitch,云台pitch轴角度值设置，向上转动方向为正方向，单位为度。
    *  @param:  short int yaw,云台pitch轴角度值设置，向右转动方向为正方向，单位为度。
    *  @return: int :错误号，0代表无错误，１代表发生错误。
    *  @note :当参数为负数时，表示负方向转动相应角度值。
    */
    void YunTaiDeltaSet(float pitch,float yaw);
    void YunTaiSmallDeltaSet(float pitch, float yaw);

    void YunTaiSmallAbsSet(float pitch, float yaw);
    void YunTaiAbsSet(float pitch,float yaw);
    /* 获取云台的绝对角度
    *　@return: Point2f，云台的pitch和yaw轴的角度　
    */
    void getAbsYunTaiDelta();

    void getAbsYunTaiSmallDelta();

    void getRelaYunTaiDelta();
   
    /** 车云台射击函数
    *  @param:  void
    *  @return: void
    */
    void YunTaiShoot(unsigned char num=0x01);
    void YunTaiSmallShoot(unsigned char num=0x01);
};



#endif
