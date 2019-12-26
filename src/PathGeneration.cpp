
#include "PathGeneration.h"
#include "random"
#include <stdint.h>
float distance(Value3 v1,Value3 v2){
	float dx = v1.X - v2.X;
	float dy = v1.Y - v2.Y;
	float dz = v1.Z - v2.Z;
	return sqrt(dx*dx+dy*dy+dz*dz);
}
enum Model
{
	dynamiccircle,
	twocircle,
	linewithcircle
};

/**
 * 第一个模型
 */
std::vector<Value3> PathGeneration(float a,float b,float c, float d,float precision){
	std::vector<Value3> path;
	Value3 tmp;
	float high = 20;
	float r;
	for(float p=-pi/4;p<pi/4;p+=precision){
		r = sqrt(std::cos(2*p));
		tmp.X=a*r*std::cos(p)+c;
		tmp.Y=b*r*std::sin(p)+d;
		tmp.Z=high;
		path.push_back(tmp);
	}
	for(float p=3*pi/4+precision;p<=5*pi/4;p+=precision){
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
	double error = 0.0;
	float x = 0.0;
	float y = 0.0;
	float x2 = 0.0;
	int size = data.size();//没有加锁，因此不能多变
	if((0<(argv[2]-argv[0]))&&
		((argv[0]+argv[2])<map_length)&&
		(0<(argv[3]-argv[1]))&&
		((argv[3]+argv[1])<map_width)){
		for(int i=0;i<size;i++){
			float minx = INT16_MAX;
			auto tmp = PathGeneration(argv[0],argv[1],argv[2],argv[3],0.5);
			for(auto &p:tmp){
				minx = std::min(minx, distance(p,data[i] ) );
			}
			error+=minx;
		}
	}else return INT16_MAX;
	return error/size;
}

//第二个模型
std::vector<Value3> PathGeneration2(float x,float y,float r, float scale,float det,float precision){
	std::vector<Value3> path;
	Value3 tmp;
	float x0 = x+r*std::cos(det);
	float y0 = y+r*std::sin(det);
	float high = 20;
	for(float i=pi;i<3*pi;i+=precision){
		tmp.X = x0+r*std::cos(i);
		tmp.Y = y0+scale*r*std::sin(i);
		tmp.Z = high;
		path.push_back(tmp);
	}
	float x1 = x-r*std::cos(det);
	float y1 = y-r*std::sin(det);
	for(float i=0;i<2*pi;i+=precision){
		tmp.X = x1+r*std::cos(i);
		tmp.Y = y1+scale*r*std::sin(i);
		tmp.Z = high;
		path.push_back(tmp);
	}
	return path;
}
float  CostPathGeneration2(std::vector<float> &argv,std::vector<Value3> &data){
	//argv[0]:x argv[1]:y argv[2]:r argv[3]:det
	double error = 0.0;
	float x = argv[0];
	float y = argv[1];
	float r = argv[2];
	float scale = argv[3];
	float det = argv[4];
	int size = data.size();//没有加锁，因此不能多变
	Value3 point1(x+r*std::cos(det),y+r*std::sin(det),20);
	Value3 point2(x-r*std::cos(det),y-r*std::sin(det),20);
	if((0<x)&&(x<map_length*2)&&(0<y)&&(y<map_length*2)&&(r<50)){
		for(int i=0;i<size;i++){
			error+=std::abs(std::min(distance(point1, data[i]),distance(point2,data[i]))-r);
		}
	}else return INT16_MAX;
	printf("%lf\n",error);
	return error/size;
}
//第三个模型
std::vector<Value3> PathGeneration3(float x,float y,float r,float det,float precision){
	std::vector<Value3> path;
	Value3 tmp;
	float high = 20;
	float lineprecision = 2*r*(std::sin(precision/2));
	for(int i=-r;i<r;i+=lineprecision){
		tmp.X=x+i/2;
		tmp.Y=y+i/2;
		tmp.Z=high;
		path.push_back(tmp);
	}
	float circle1x = x + std::sqrt(2)*r;
	float circle1y = y ;
	for(int i=-3*pi/4;i<3*pi/4;i+=precision){
		tmp.X= circle1x+r*std::cos(i);
		tmp.Y= circle1y+r*std::sin(i);
		tmp.Z= high;
		path.push_back(tmp);
	}
	for(int i=-r;i<r;i+=lineprecision){
		tmp.X=x-i/2;
		tmp.Y=y-i/2;
		tmp.Z=high;
		path.push_back(tmp);
	}
	float circle2x = x - std::sqrt(2)*r;
	float circle2y = y ;
	for(int i=-pi/4;i>-7*pi/4;i-=precision){
		tmp.X= circle2x+r*std::cos(i);
		tmp.Y= circle2y+r*std::sin(i);
		tmp.Z= high;
		path.push_back(tmp);
	}
	return path;
}
float  CostPathGeneration3(std::vector<float> &argv,std::vector<Value3> &data){
	//argv[0]:a argv[1]:b argv[2]:c argv[3]:d
	double error = 0.0;
	float x = 0.0;
	float y = 0.0;
	float x2 = 0.0;
	int size = data.size();//没有加锁，因此不能多变
	if((0<(argv[2]-argv[0]))&&
		((argv[0]+argv[2])<map_length)&&
		(0<(argv[3]-argv[1]))&&
		((argv[3]+argv[1])<map_width)){
		for(int i=0;i<size;i++){
			float minx = INT16_MAX;
			auto tmp = PathGeneration3(argv[0],argv[1],argv[2],argv[3],0.5);
			for(auto &p:tmp){
				minx = std::min(minx, distance(p,data[i] ) );
			}
			error+=minx;
		}
	}else return INT16_MAX;
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
