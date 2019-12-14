////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      SerialPort Code for robot
///ALL RIGHTS RESERVED
///@file:serial_port_debug.h
///@brief: 串口调试模块。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-5-19
///修订历史：
////////////////////////////////////////////////////////////////////////////////

#ifndef RADIODEBUG_H
#define RADIO_DEBUG_H

#include "radio_interface.h"
#include "radio_packet.h"

class RadioPortDebug{
public:
    RadioPortDebug();
    ~RadioPortDebug();
public:
    void init(RadioInterface *serialInterface);
    void testSerialPort();
    static void testSerialPortListenPrint(RadioPacket recvPacket);

private:
    RadioInterface *pSerialInterface;
};

#endif
