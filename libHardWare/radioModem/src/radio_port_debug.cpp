////////////////////////////////////////////////////////////////////////////////
///Copyright(c)     UESTC ROBOMASTER2018      SerialPort Code for robot
///ALL RIGHTS RESERVED
///@file:serial_port_debug.cpp
///@brief: 机器人串口调试模块
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-5-19
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#include "radio_port_debug.h"

using namespace std;

RadioPortDebug::RadioPortDebug() {

}

RadioPortDebug::~RadioPortDebug() {

}

void RadioPortDebug::init(RadioInterface *serialInterface) {
    pSerialInterface=serialInterface;
}

void RadioPortDebug::testSerialPort() {
    char c=getchar();
    cout<<"start to test serial port,press 'c' to continue,press 'q' to quit!"<<endl;
    for(int i = 0 ; i<10;i++){
        pSerialInterface->YunTaiDeltaSet(5,0);
    }
    
}

