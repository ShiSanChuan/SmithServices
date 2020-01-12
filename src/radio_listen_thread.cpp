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
            unsigned char device =  recvPacket.getUncharInBuffer(2);
            if(device>Device_size){
                std::cerr<<"error data?"<<std::endl;
            }else{
                std::lock_guard<std::mutex> lock(rmut);
                devices[device].Posion.X =  recvPacket.getFloatInBuffer(3);
                devices[device].Posion.Y =  recvPacket.getFloatInBuffer(7);
                devices[device].Posion.Z =  recvPacket.getFloatInBuffer(11);
                devices[device].situation =  CMD;
                devices[device].update = true;                 
            }
             
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
    assert(ID==data.ID);
    RadioPacket sendPacket;
    sendPacket.creatPacket(data.situation);
    sendPacket.setUncharInBuffer(data.ID,2);
    sendPacket.setFloatInBuffer(data.Posion.X, 3);
    sendPacket.setFloatInBuffer(data.Posion.Y, 7);
    sendPacket.setFloatInBuffer(data.Posion.Z, 11);
    Radio.dataSend(sendPacket);
}