#ifndef _SOLVE_H
#define _SOLVE_H

#include "common.h"
#include "thread_pool.h"
#include <ceres/ceres.h>
#include "opencv2/opencv.hpp"
#include <vector>
#include <functional>
#include <map>

enum Solve
{	
	GaSolve,
	CeresSolve
};

//特殊版
class GA

{
public:
	float Accuracy;
	float min_Accuracy;
	std::vector<Value3> data;
	bool rebuild;
private:
	int chrom_num;
	int para_num;
	float c1;
	float c2;
	float k;
	float w;
	cv::Mat speed;
	cv::Mat Population;
	cv::Mat Pbest;//历史最佳
	std::vector<float> Gbest;//globel best 全局最优
	std::vector<float> post;
	std::vector<float> ost;
	std::vector<float> paramter;
	float (*fun)(std::vector<float> &argv,std::vector<Value3> &data);
	float dmin;
	float dmax;
	
	std::vector<float> min_Gbest;
public:
	GA(int para_num,float dmin,float dmax,float(*_fun)(std::vector<float> &argv,std::vector<Value3> &data));
	void Setthread(ThreadPool &pool);
	void addPoint(Value3 point);
	std::vector<float> GetOptimal();
	~GA();
public:
	void init();
	void ranking();
	void update(bool para=0);
};
class AnalyticCostFunctor : public ceres::SizedCostFunction<1,4> {
public:
 AnalyticCostFunctor(const double x, const double y) : data_x(x), data_y(y) {}
 virtual ~AnalyticCostFunctor() {}
 virtual bool Evaluate(double const* const* parameters,
                       double* residuals,
                       double** jacobians) const {
        const double a = parameters[0][0];
        const double b = parameters[0][1];
        const double c = parameters[0][2];
        const double d = parameters[0][3];
        const double theta = atan2((data_y-d)*a, (data_x-c)*b);
        double ro;
        if(cos(theta*2) > 0)
          ro = sqrt(cos(theta*2));
        else
          {
            ro = 5;
          }
        // cout << "ro: " << ro << endl;  
        // 残差矩阵
        residuals[0] = pow(data_y-b*ro*sin(theta)-d,2) + pow(data_x-a*ro*cos(theta)-c,2);
        if (!jacobians) return true;
        double*jacobian = jacobians[0];
        if (!jacobian) return true;
        // 雅可比矩阵
        jacobian[1] = -ro*sin(theta)*(data_y - b*ro*sin(theta)-d)*2;
        jacobian[0] = -ro*cos(theta)*(data_x - a*ro*cos(theta)-c)*2;
        jacobian[2] = -2*(data_x - a*ro*cos(theta)-c);
        jacobian[3] = -2*(data_y - b*ro*sin(theta)-d);
        return true;
 }

private:
	const double data_x;
	const double data_y;
 };
class Ceres
{
private:

private:
	

public:
	float Accuracy;
	bool rebuild;
	std::vector<Value3> data;
	std::vector<float> Gbest;
public:
	Ceres();
	~Ceres();
	void Setthread(ThreadPool &pool);
	void addPoint(Value3 point);
	std::vector<float> GetOptimal();
};

class FactorySolve
{
private:
	static GA* GASolver;
	static Ceres* CeresSolver;
	FactorySolve();
public:
	static GA* addSolve(Solve solve,int para_num,
		float dmin,float dmax,
		float(*_fun)(std::vector<float> &argv,std::vector<Value3> &data));
	static Ceres* addSolve(Solve solve);
	static void* getSolve(Solve solve);
	static void close();
	~FactorySolve();
};

#endif