#ifndef _COMMON_H
#define _COMMON_H
#include<stdint.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <cmath>
#include <assert.h>


#define Device_size 9
#define pi 3.1415926

//Size
#define map_width 40
#define map_length 100

//UAV
#define UAV_filed 11.54 //视野范围
#define UAV_speed 0.006 //速度

#define BALLON_size 6
extern int flag;

typedef enum:unsigned char{
	ROBOT_MODE_IN_INIT = 0x00,//初始化启动
	ROBOT_MODE_IN_RETURN = 0x10,//返回
	ROBOT_MODE_IN_MOVE = 0x20,//移动
	ROBOT_MODE_IN_STAB = 0x40,//刺气球
	ROBOT_MODE_IN_CATCH = 0x80,//抓气球	
}Status;

typedef enum:unsigned char
{
	Service = 0x00,
	UAV1 = 0x01,
	UAV2 = 0x02,
	UAV3 = 0x04,
	AIM = 0x08
}Marker;

typedef struct Value3
{
	Value3(float X=0,float Y=0,float Z=0){
		this->X=X;
		this->Y=Y;
		this->Z=Z;
	}
	float X;
	float Y;
	float Z;
}Value3;

typedef struct UAV
{
	UAV(uint8_t situation=0){
		this->situation = situation;
		this->update = 0;
		this->Posion.X = 0;
		this->Posion.Y = 0;
		this->Posion.Z = 0;
	}
	Value3 Posion;
	uint8_t situation;
	bool update;
}UAV;

typedef struct LinkList
{
	LinkList(Value3 &data){//必须需要数来初始化
		this->data = data;
		this->next = nullptr;
	}
	Value3 data;
	LinkList * next;
}LinkList;


#endif
