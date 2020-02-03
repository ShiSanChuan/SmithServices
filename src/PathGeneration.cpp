#include <Solve.h>
#include "PathGeneration.h"
#include "random"
#include <stdint.h>
#include <algorithm>
double distance(Value3 v1,Value3 v2){
	double dx = v1.X - v2.X;
	double dy = v1.Y - v2.Y;
	double dz = v1.Z - v2.Z;
	return sqrt(dx*dx+dy*dy+dz*dz);
}
enum Model
{
	dynamiccircle,
	twocircle,
	linewithcircle
};
LinkList *Pathline(Value3 start,int line_size,double precision){
	LinkList * head = new LinkList(start);
	LinkList * N = head;
	LinkList * M;
	Value3 tmp;
	tmp.X = start.X;
	tmp.Y = start.Y;
	tmp.Z = start.Z;
	std::vector<LinkList*> reverse_path;
	for(double i=0;i<line_size;i+=precision){
		tmp.Y +=precision;
		M = new LinkList(tmp);
		reverse_path.push_back( new LinkList(tmp));
		N->next = M;
		N = N->next;
	}
	reverse(reverse_path.begin(),reverse_path.end());
	for(auto &p:reverse_path){
		N->next = p;
		N = N->next;
	}
	N->next = head;
	return head;
}
LinkList * Patharch(Value3 start,int curve,double precision){
    LinkList * head = new LinkList(start);
    LinkList * N = head;
    LinkList * M;
    Value3 tmp;
    tmp.X = start.X;
    tmp.Y = start.Y;
    tmp.Z = start.Z;
    std::vector<LinkList*> reverse_path;
    for(int i = 0;i<=curve;){
        if(tmp.Y  > map_width){
            i++;
            if(i>curve)break;
            precision = -std::abs(precision);
            tmp.Y = map_width;
            for(int j=0;j<10;j++){
                tmp.X +=  map_length/(curve*2*10);
                M = new LinkList(tmp);
                reverse_path.push_back( new LinkList(tmp));//避免指针混乱
                N->next = M;
                N = N->next;
            }
            
        }else if(tmp.Y < 0){
            i++;
            if(i>curve)break;
            precision = std::abs(precision);
            tmp.Y = 0;
            for(int j=0;j<10;j++){
                tmp.X +=  map_length/(curve*2*10);
                M = new LinkList(tmp);
                reverse_path.push_back( new LinkList(tmp));//避免指针混乱
                N->next = M;
                N = N->next;
            }
        }
        tmp.Y = tmp.Y + precision;
        M = new LinkList(tmp);
        reverse_path.push_back( new LinkList(tmp));//避免指针混乱
        N->next = M;
        N = N->next;
    }
    // reverse(reverse_path.begin(),reverse_path.end());
    // for(auto &p:reverse_path){
    //     N->next = p;
    //     N = N->next;
    // }
    // N->next = head;
    return head;
}

/**
 * 第一个模型
 */
std::vector<Value3> PathGeneration(double a,double b,double c, double d,double precision){
	std::vector<Value3> path;
	Value3 tmp;
	double high = 20;
	double r;
	for(double p=-pi/4;p<pi/4;p+=precision){
		r = sqrt(std::cos(2*p));
		tmp.X=a*r*std::cos(p)+c;
		tmp.Y=b*r*std::sin(p)+d;
		tmp.Z=high;
		path.push_back(tmp);
	}
	for(double p=3*pi/4+precision;p<=5*pi/4;p+=precision){
		r = sqrt(std::cos(2*p));
		tmp.X=a*r*std::cos(p)+c;
		tmp.Y=-b*r*std::sin(p)+d;
		tmp.Z=high;
		path.push_back(tmp);
	}
	return path;
}
double  CostPathGeneration(std::vector<double> &argv,std::vector<Value3> &data){
	//argv[0]:a argv[1]:b argv[2]:c argv[3]:d
	double error = 0.0;
	double x = 0.0;
	double y = 0.0;
	double x2 = 0.0;
	int size = data.size();//没有加锁，因此不能多变
	if((0<(argv[2]-argv[0]))&&
		((argv[0]+argv[2])<map_length)&&
		(0<(argv[3]-argv[1]))&&
		((argv[3]+argv[1])<map_width)){
		for(int i=0;i<size;i++){
			double minx = INT16_MAX;
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
std::vector<Value3> PathGeneration2(double x,double y,double r, double scale,double det,double precision){
	std::vector<Value3> path;
	Value3 tmp;
	double x0 = x+r*std::cos(det);
	double y0 = y+r*std::sin(det);
	double high = 20;
	for(double i=pi;i<3*pi;i+=precision){
		tmp.X = x0+r*std::cos(i);
		tmp.Y = y0+scale*r*std::sin(i);
		tmp.Z = high;
		path.push_back(tmp);
	}
	double x1 = x-r*std::cos(det);
	double y1 = y-r*std::sin(det);
	for(double i=0;i<2*pi;i+=precision){
		tmp.X = x1+r*std::cos(i);
		tmp.Y = y1+scale*r*std::sin(i);
		tmp.Z = high;
		path.push_back(tmp);
	}
	return path;
}
double  CostPathGeneration2(std::vector<double> &argv,std::vector<Value3> &data){
	//argv[0]:x argv[1]:y argv[2]:r argv[3]:det
	double error = 0.0;
	double x = argv[0];
	double y = argv[1];
	double r = argv[2];
	double scale = argv[3];
	double det = argv[4];
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
std::vector<Value3> PathGeneration3(double r,double c,double d,double det,double precision){
	std::vector<Value3> path;
	Value3 tmp;
	double high = 20;
	double speed = 2;
	double sampleTime = 0.1;
	double T = 2*r/speed;
	double theta1;
    for(double t = 0; t < T; t=t+0.1){
    	tmp.X = -sqrt(2)/2*r + sqrt(2)/2*speed*t + c;
    	tmp.Y = -sqrt(2)/2*r + sqrt(2)/2*speed*t + d;
    	tmp.Z = high;
    	path.push_back(tmp);
    }
    T = 3*pi*r/2/speed;
    for(double t = 0; t < T; t=t+0.1){
      theta1 = 3*pi/4 - speed*t/r;
      tmp.X = sqrt(2)*r + r*cos(theta1) + c;
      tmp.Y = r*sin(theta1) + d;
      tmp.Z = high;
      path.push_back(tmp);
    }    
    T = 2*r/speed;
    for(double t = 0; t < T; t=t+0.1){
      theta1 = 3*pi/4 - speed*t/r;
      tmp.X = sqrt(2)/2*r - sqrt(2)/2*speed*t + c;
      tmp.Y = -sqrt(2)/2*r + sqrt(2)/2*speed*t + d;
      tmp.Z = high;
      path.push_back(tmp);
    }
    T = 3*pi*r/2/speed;
    for(double t = 0; t < T; t=t+0.1){
      theta1 = pi/4 + speed*t/r;
      tmp.X = -sqrt(2)*r + r*cos(theta1) + c;
      tmp.Y = r*sin(theta1) + d;
      tmp.Z = high;
      path.push_back(tmp);
    }    
	return path;
}
double  CostPathGeneration3(std::vector<double> &argv,std::vector<Value3> &data){
	//argv[0]:a argv[1]:b argv[2]:c argv[3]:d
	// if(arg)
	if((argv[0]>= argv[2])||(argv[0]>=map_width-argv[2]))
		return INT16_MAX;
	ceres::Problem problem;	
	double param[3] = {0};
	param[0] = argv[0];
	param[1] = argv[1];
	param[2] = argv[2];
	for(int i=0;i<data.size();i++){
	    ceres::CostFunction *cost_function = new AnalyticCostFunctor(data[i].X,data[i].Y);
	    problem.AddResidualBlock(cost_function, NULL, param);
	}
	ceres::Solver::Options options;
	options.gradient_tolerance = 1e-32;
	options.function_tolerance = 1e-16;
	options.linear_solver_type = ceres::DENSE_QR;
	options.max_num_iterations = 100;
	static std::string error = "error ceres";
	options.IsValid(&error);
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	argv[0] = param[0];
	argv[1] = param[1];
	argv[2] = param[2];
	if(summary.final_cost<0)return INT16_MAX;
	return summary.final_cost;
}

std::vector<Value3> BallonGeneration(int width,int length){
	std::vector<Value3> Point;
	Value3 tmp;
	double high = 10;
	for(int i=0;i<BALLON_num;i++){
		tmp.X = rand()%length;
		tmp.Y = rand()%width;
		tmp.Z = high;
		Point.push_back(tmp);
	}
	return Point;
}
