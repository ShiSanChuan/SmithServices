
#include "PathGeneration.h"
#include "random"
#include <stdint.h>
float distance(Value3 v1,Value3 v2){
	float dx = v1.X - v2.X;
	float dy = v1.Y - v2.Y;
	float dz = v1.Z - v2.Z;
	return sqrt(dx*dx+dy*dy+dz*dz);
}
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
	float tmp1;
	float tmp2;
	float tmp3;
	float tmp4;
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
			// if((argv[2]-argv[0]<=data[i].X&&(data[i].X<=(argv[0]+argv[2])))){
			// 	// &&((argv[3]-argv[1]*0.3536)<data[i].Y&&(data[i].Y<(argv[3]+argv[1]*0.3536)))){//上下界
			// 	y = (data[i].Y-argv[3])/argv[1];
			// 	x = (data[i].X-argv[2])/argv[0];
			// 	x2 = x*x;
			// 	tmp1 = std::sqrt(1+8*x2);
			// 	tmp2 = (-2*x2-1 + tmp1)/2;
			// 	if(tmp2>0){
			// 		tmp3 = std::sqrt(tmp2);
			// 		tmp4 = (tmp3-x)* argv[0]+argv[2];//还原误差
			// 		if(minx>tmp4*tmp4){//最小误差
			// 			minx = tmp4*tmp4;
			// 		}
			// 		tmp4 = (-tmp3-x)* argv[0]+argv[2];
			// 		if(minx>tmp4*tmp4 ){
			// 			minx = tmp4*tmp4;
			// 		}
			// 	}
			// 	error+=minx;
			// }else{
				// y = (data[i].Y-argv[3]);
				// x = (data[i].X-argv[2]);
				// error += std::sqrt(x*x+y*y);
			// }
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
