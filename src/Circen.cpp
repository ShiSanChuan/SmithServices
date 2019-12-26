#include "Solve.h"
#include "PathGeneration.h"
#include <mutex>
#include <cmath>
static std::mutex mut;

Circen::Circen(float precision){
	Accuracy = INT16_MAX;
	this->precision = precision;
}

Circen::~Circen(){

}

void Circen::Setthread(ThreadPool &pool){

}

void Circen::addPoint(Value3 point){
	if(data.size()&&(distance(data.back(),point)>1)){
		data.push_back(point);
		float r;
		printf("---------------------------------------\n");
		std::vector<float> argv{0,0,0,0,0};
		if(data.size()>5){
			for(int i = data.size()-1;i>=3;i--){
				int j = i/2;
				// while(j==0)j=rand()%(i-1);
				int m = j/2;
				Value3 center = CalculaCenter(data[i],data[j],data[m],r);
				if(std::abs(r+1)<0.00001)continue;
				argv[0] = center.X - r;
				argv[1] = center.Y;
				argv[2] = r;
				argv[3] = 1;
				argv[4] = 0;
				float acc = CostPathGeneration2(argv,data);
				printf("%f %f %f %f\n",center.X,center.Y,r,acc);
				if(acc<Accuracy){
					Gbest = argv;
					Accuracy = acc;
				}
			}
		}
	}else if(!data.size()){
		data.push_back(point);
	}
}

std::vector<float> Circen::GetOptimal(){
	return Gbest;
}

Value3 Circen::CalculaCenter(Value3 &point1,Value3 &point2,Value3 &point3,float &mid){
	double a = point1.X-point2.X;
	double b = point1.Y-point2.Y;
	double c = point1.X-point3.X;
	double d = point1.Y-point3.Y;
	double det = b*c - a*d;
	double e = ((point1.X*point1.X-point2.X*point2.X)+
				(point1.Y*point1.Y-point2.Y*point2.Y))/2.0;
	double f = ((point1.X*point1.X-point3.X*point3.X)+
				(point1.Y*point1.Y-point3.Y*point3.Y))/2.0;
	if(fabs(det)<1e-5){
		mid = -1;
		return Value3(0,0,0);
	}
	double x0 = -(d*e-b*f)/det;
	double y0 = -(a*f-c*e)/det;
	mid = hypot(point1.X-x0, point1.Y-y0);
	return Value3(x0,y0,20); 
}
double Circen::distance(Value3 v1, Value3 v2){
	float dx = v1.X - v2.X;
	float dy = v1.Y - v2.Y;
	return sqrt(dx*dx+dy*dy);
}