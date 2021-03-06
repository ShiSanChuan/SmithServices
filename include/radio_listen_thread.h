#ifndef RADIO_LISTEN_THREAD_H
#define RADIO_LISTEN_THREAD_H
#include "common.h"
#include <string>
#include "radio_interface.h"
#include "thread_pool.h"
#include "radio_packet.h"

class RadioListen{
private:
	static UAV devices[Device_size];

	static RadioInterface Radio[UAV_size];
public:
	RadioListen(ThreadPool &pool,std::vector<std::string> port);
	~RadioListen();
	UAV GetUAVData(Marker ID);
	Value3 GetUAVPosion(Marker ID);
	uint8_t GetUAVCommend(Marker ID);
	void SetUAVData(Marker ID,UAV &data);
	bool CheckUAV(Marker ID);
};

#endif
