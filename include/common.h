#ifndef _COMMON_H
#define _COMMON_H
#include<stdint.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <cmath>
#include <assert.h>
#include <stdint.h>
#include "yaml-cpp/yaml.h"

#define Device_size 9
extern float pi ;

//Size
extern float map_width ;
extern float map_length;
//UAV
extern float UAV_filed ;//视野范围
extern float UAV_speed ; //速度
extern float Simulate_speed ;
extern int BALLON_num ;
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

namespace YAML {//解析用
	template<>
	struct convert<Value3> {
	  static Node encode(const Value3& rhs) {
	    Node node;
	    node.push_back(rhs.X);
	    node.push_back(rhs.Y);
	    node.push_back(rhs.Z);
	    return node;
	  }

	  static bool decode(const Node& node, Value3& rhs) {
	    if(!node.IsSequence() || node.size() != 3) {
	      return false;
	    }

	    rhs.X = node[0].as<double>();
	    rhs.Y = node[1].as<double>();
	    rhs.Z = node[2].as<double>();
	    return true;
	  }
	};
}


#endif
