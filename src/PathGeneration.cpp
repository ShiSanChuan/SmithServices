
#include "PathGeneration.h"
#include "random"

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

std::vector<Value3> BallonGeneration(int width,int length){
	std::vector<Value3> Point;
	Value3 tmp;
	float high = 10;
	for(int i=0;i<BALLON_size;i++){
		tmp.X = rand()%length;
		tmp.Y = rand()%width;
		tmp.Z = high;
		Point.push_back(tmp);
	}
	return Point;
}