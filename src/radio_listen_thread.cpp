#include "radio_listen_thread.h"
#include <mutex>

UAV RadioListen::devices[Device_size];
RadioInterface RadioListen::Radio;
static std::mutex rmut;
static std::mutex wmut;
static void run(RadioInterface *Radio,UAV *devices){
    Value3 data;
    RadioPacket recvPacket;
    while(flag){
        if(Radio->dataRecv(recvPacket)==0){
            unsigned char CMD = recvPacket.getCMD();
            if((CMD&0xf0)>0x80){//命令
                std::cerr<<"unknow command"<<std::endl;
            }else{//数据
                unsigned char device =  CMD&0x0f;//设备
                if(device>Device_size){
                    std::cerr<<"error data?"<<std::endl;
                }else{
                    std::lock_guard<std::mutex> lock(rmut);
                    devices[device].Posion.X =  recvPacket.getFloatInBuffer(2);
                    devices[device].Posion.Y =  recvPacket.getFloatInBuffer(6);
                    devices[device].Posion.Z =  recvPacket.getFloatInBuffer(10);
                    devices[device].situation =  (CMD&0xf0);
                    devices[device].update = true;                 
                }
            } 
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    flag = false;
}

bool RadioListen::CheckUAV(Marker ID){
    bool data = devices[ID].update;
    devices[ID].update =false;
    return data;
}

RadioListen::RadioListen(ThreadPool &pool ,std::string port){
    Radio.init(port);//"/dev/ttyUSB0"
    if(!Radio.isOpen()){
        std::cerr<<"error open uart "<<port<<std::endl;
        flag = false;
    }
    pool.enqueue(run,&this->Radio,this->devices);
    return ;
}

RadioListen::~RadioListen(){

}


UAV RadioListen::GetUAVData(Marker ID){
    UAV device;
    {
        std::lock_guard<std::mutex> lock(rmut);
        device = devices[ID];
    }
    return  device;
}
Value3 RadioListen::GetUAVPosion(Marker ID){
    Value3 data;
    {
        std::lock_guard<std::mutex> lock(rmut);
        data = devices[ID].Posion;
    }
    return data;
}

uint8_t RadioListen::GetUAVCommend(Marker ID){
    uint8_t data;
    {
        std::lock_guard<std::mutex> lock(rmut);
        data = devices[ID].situation;
    }
    return data;    
}
void RadioListen::SetUAVData(Marker ID,UAV &data){
    std::lock_guard<std::mutex> lock(wmut);
    RadioPacket sendPacket;
    sendPacket.creatPacket(data.situation);
    sendPacket.setFloatInBuffer(data.Posion.X, 2);
    sendPacket.setFloatInBuffer(data.Posion.Y, 6);
    sendPacket.setFloatInBuffer(data.Posion.Z, 10);
    Radio.dataSend(sendPacket);
}