
#include "PathGeneration.h"
#include "random"
#include <stdint.h>
std::vector<Value3> PathGeneration(float a,float b,float c, float d){
	std::vector<Value3> path;
	Value3 tmp;
	float high = 20;
	float r;
	for(float p=-pi/4;p<pi/4;p+=0.01){
		r = sqrt(std::cos(2*p));
		tmp.X=a*r*std::cos(p)+c;
		tmp.Y=b*r*std::sin(p)+d;
		tmp.Z=high;
		path.push_back(tmp);
	}
	for(float p=3*pi/4+0.01;p<=5*pi/4;p+=0.01){
		r = sqrt(std::cos(2*p));
		tmp.X=a*r*std::cos(p)+c;
		tmp.Y=-b*r*std::sin(p)+d;
		tmp.Z=high;
		path.push_back(tmp);
	}
	return path;
}

float  CostPathGeneration(std::vector<float> &argv,std::vector<Value3> &data){
	//argv[0]:a argv[1]:b argv[2]:c argv[3]:d
	float error = 0.0;
	float x = 0.0;
	float y = 0.0;
	float y2 = 0.0;
	float tmp1;
	float tmp2;
	float tmp3;
	float tmp4;
	int size = data.size();//没有加锁，因此不能多变
	for(int i=0;i<size;i++){
		float minx = INT16_MAX;
		y = (data[i].Y-argv[3])/argv[1];
		x = (data[i].X-argv[2])/argv[0];
		y2 = y*y;
		tmp1 = std::sqrt(1-8*y*y);
		tmp2 = 1-2*y*y - tmp1;
		tmp3 = 1-2*y*y + tmp1;
		if(tmp2>0){
			tmp4 = std::sqrt(tmp2/2);
			if(minx>std::abs(tmp4-x)){
				minx = std::abs(tmp4-x);
			}
			if(minx>std::abs(-tmp4-x) ){
				minx = std::abs(-tmp4-x);
			}
		}
		if(tmp3>0){
			tmp4 = std::sqrt(tmp3/2);
			if(minx>std::abs(tmp4-x)){
				minx = std::abs(tmp4-x);
			}
			if(minx>std::abs(-tmp4-x) ){
				minx = std::abs(-tmp4-x);
			}
		}
		if(minx!=INT16_MAX)
			error+=minx;
	}
	return error/size;
}

std::vector<Value3> BallonGeneration(int width,int length){
	std::vector<Value3> Point;
	Value3 tmp;
	float high = 10;
	for(int i=0;i<BALLON_num;i++){
		tmp.X = rand()%length;
		tmp.Y = rand()%width;
		tmp.Z = high;
		Point.push_back(tmp);
	}
	return Point;
}
float distance(Value3 v1,Value3 v2){
	float dx = v1.X - v2.X;
	float dy = v1.Y - v2.Y;
	float dz = v1.Z - v2.Z;
	return sqrt(dx*dx+dy*dy+dz*dz);
}