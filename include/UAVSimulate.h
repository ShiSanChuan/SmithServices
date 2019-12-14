#ifndef _UAVSIMULATE_H
#define _UAVSIMULATE_H
#include "common.h"
#include "radio_listen_thread.h"
#include "thread_pool.h"
#include "timer.h"
#include <map>
class UAVSimulate
{
public:
	UAV uav;
	Value3 UAV_AIM;
	std::vector<Value3> path;
public:
	void init(ThreadPool &pool,RadioInterface *Radio);//追寻机 按照命令和查找动作运动
	void init(ThreadPool &pool,RadioInterface *Radio,std::vector<Value3> path);//沿轨迹运动 目标机
	void update();
	void SetPosion(Value3 data);
	void SetSituation(uint8_t situation);
};

class Simulate
{
public:
	static RadioInterface Radio;
	static UAVSimulate* Worker[Device_size];
private:
	Simulate();
public:
	static void init(ThreadPool &pool,std::string port="/dev/ttyUSB1");
	static UAVSimulate * addUAV(Marker ID);
	static UAVSimulate* getUAV(Marker ID);
	void close();
	~Simulate();
	
};

#endif

