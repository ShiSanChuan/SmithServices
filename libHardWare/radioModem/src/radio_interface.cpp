////////////////////////////////////////////////////////////////////////////////
///Copyright(c)     UESTC ROBOMASTER2018      SerialPort Code for robot
///ALL RIGHTS RESERVED
///@file:serial_interface.cpp
///@brief: 机器人控制基本接口源文件，包含对车底盘及云台的基本接口。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-3-4
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#include "radio_interface.h"

RadioInterface::RadioInterface(){

};


RadioInterface::~RadioInterface(void) {

};

int RadioInterface::init(std::string devPath){
    if(mMcuSerialPort.init(devPath)){
        return 0;
    }else{
        return -1;
    }
}

bool RadioInterface::isOpen(){
    return mMcuSerialPort.isOpen();
}

int RadioInterface::dataSend(RadioPacket sendPacket){
    if(mMcuSerialPort.Send(sendPacket.buffer,16)==16){
        return 0;
    }
    return -1;

}
int RadioInterface::dataRecv(RadioPacket &recvPacket){

    if(mMcuSerialPort.Recv(recvPacket.buffer,16)==16) {
        if(recvPacket.unPacking()==0){
            return 0;
        }
    }
    return -1;
}
int RadioInterface::dataSend32(RadioPacket sendPacket){
    if(mMcuSerialPort.Send(sendPacket.buffer32,32)==32){
        return 0;
    }
    return -1;

}
int RadioInterface::dataRecv32(RadioPacket &recvPacket){

    if(mMcuSerialPort.Recv(recvPacket.buffer32,32)==32) {
        if(recvPacket.unPacking32()==0){
            return 0;
        }
    }
    return -1;
}


/*********************控制接口**************************/




