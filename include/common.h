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

#define Device_size 5
#define UAV_size 3
#define Status_size 256
typedef enum:unsigned char{
	ROBOT_MODE_IN_INIT    = 0x00,//初始化
	ROBOT_MODE_IN_TAKEOFF = 0x01,//自动起飞
	ROBOT_MODE_IN_MOVETO  = 0x02,//飞到一个点
	ROBOT_MODE_IN_LINE    = 0x03,//一条线飞
	ROBOT_MODE_IN_ARCH    = 0x04,//弓字型飞
	ROBOT_MODE_IN_CATCH   = 0x05,//抓气球
	ROBOT_MODE_IN_STAB    = 0x06,//刺气球
	ROBOT_MODE_IN_RETURN  = 0x07,//返回
	ROBOT_MODE_IN_LOST    = 0xfe,//时延太高丢失
	ROBOT_MODE_IN_EMPTY   = 0xff,//空状态	
}Status;

typedef enum:unsigned char
{
	UAV1 = 0x00,
	UAV2 = 0x01,
	UAV3 = 0x02,
	AIM = 0x03,
	Service = 0x04
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
	UAV(uint8_t situation=0xff,uint8_t ID=0xff){
		this->situation = situation;
		this->ID = ID;
		this->update = 0;
		this->Posion.X = 0;
		this->Posion.Y = 0;
		this->Posion.Z = 0;
	}
	Value3 Posion;
	uint8_t situation;
	uint8_t ID;
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
};

#define PNONE                 "\e[0m"
#define PRED                  "\e[1;31m"
#define PGREEN                "\e[1;32m"
#define PYELLOW               "\e[1;33m"
#define PWHITE                "\e[1;37m"

extern float pi ;

//Size
extern float map_width ;
extern float map_length;
//UAV
extern float UAV_filed ;//视野范围
extern const float UAV_low_filed ;
extern float UAV_speed ; //速度
extern float Simulate_speed ;
extern Value3 BALLON_Posion[6];
extern int BALLON_num ;
extern int flag;

#endif
