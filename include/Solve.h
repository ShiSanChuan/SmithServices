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
	CeresSolve,
	CircenSolve
};

//特殊版
class GA

{
public:
	double Accuracy;
	double min_Accuracy;
	std::vector<Value3> data;
	bool rebuild;
private:
	int chrom_num;
	int para_num;
	double c1;
	double c2;
	double k;
	double w;
	cv::Mat speed;
	cv::Mat Population;
	cv::Mat Pbest;//历史最佳
	std::vector<double> Gbest;//globel best 全局最优
	std::vector<double> post;
	std::vector<double> ost;
	std::vector<double> paramter;
	double (*fun)(std::vector<double> &argv,std::vector<Value3> &data);
	double dmin;
	double dmax;
	
	std::vector<double> min_Gbest;
public:
	GA(int para_num,double dmin,double dmax,double(*_fun)(std::vector<double> &argv,std::vector<Value3> &data));
	void Setthread(ThreadPool &pool);
	void addPoint(Value3 point);
	std::vector<double> GetOptimal();
	~GA();
public:
	void init();
	void ranking();
	void update(bool para=0);
};
class AnalyticCostFunctor : public ceres::SizedCostFunction<1,3> {
public:
 AnalyticCostFunctor(const double x, const double y) : data_x(x), data_y(y) {}
 virtual ~AnalyticCostFunctor() {}
 virtual bool Evaluate(double const* const* parameters,
                       double* residuals,
                       double** jacobians) const {
            const double r = parameters[0][0];
            const double c = parameters[0][1];
            const double d = parameters[0][2];
            // 残差矩阵
            //场地是一个100*40的有限场地，因此存在约束条件。
            // 1) r + sqrt(2)*r + c < 100, 2) c - r - sqrt(2)*r > 0
            // 3) d + r < 40, 4) d - r > 0
            // 采用惩罚函数法，当不满足约束条件是残差十分大。
            // 判断是曲线还是直线
            if(r + sqrt(2)*r + c < 100 && c - r - sqrt(2)*r > 0 && d + r < 40 && d - r > 0)
            {
              //如果在圆心右侧
              if(data_x > c)
              {
                double theta1 = atan2(data_y - d, data_x - sqrt(2)*r - c);
                //残差矩阵
                residuals[0] = pow(data_y - d - r*sin(theta1) ,2) + pow(data_x - r*cos(theta1) - sqrt(2)*r - c,2);
                if (!jacobians) return true;
                double*jacobian = jacobians[0];
                if (!jacobian) return true;
                // 雅可比矩阵
                jacobian[0] = -sin(theta1)*(data_y - d - r*sin(theta1))*2 - (cos(theta1) + sqrt(2))*(data_x - r*cos(theta1) - sqrt(2)*r - c)*2;
                jacobian[1] = -(data_x - r*cos(theta1) - sqrt(2)*r - c)*2;
                jacobian[2] = -(data_y - d - r*sin(theta1))*2;
              
              }
              else
              {
                double theta2 = atan2(data_y - d, data_x + sqrt(2)*r - c);
                residuals[0] = pow(data_y - d - r*sin(theta2) ,2) + pow(data_x - r*cos(theta2) + sqrt(2)*r - c,2);
                if (!jacobians) return true;
                double*jacobian = jacobians[0];
                if (!jacobian) return true;
                // 雅可比矩阵
                jacobian[0] = -sin(theta2)*(data_y - d - r*sin(theta2))*2 - cos(theta2) - sqrt(2);
                jacobian[1] = -(data_x - r*cos(theta2) + sqrt(2)*r - c)*2;
                jacobian[2] = -(data_y - d - r*sin(theta2))*2;
              }
            }
            else
            {
              residuals[0] = 1000;
              residuals[1] = 1000;
              residuals[2] = 1000;
             
            }
            return true;
 }

private:
	const double data_x;
	const double data_y;
 };
 struct ExponentialResidual {
  ExponentialResidual(double x, double y)
      : x_(x), y_(y) {}

  template <typename T>
  bool operator()(const T* const m, const T* const c, T* residual) const {
    residual[0] = T(y_) - exp(m[0] * T(x_) + c[0]);
    return true;
  }

 private:
  // Observations for a sample.
  const double x_;
  const double y_;
};

class Ceres
{
private:

private:
	

public:
	double Accuracy;
	bool rebuild;
	std::vector<Value3> data;
	std::vector<double> Gbest;
public:
	Ceres();
	~Ceres();
	void Setthread(ThreadPool &pool);
	void addPoint(Value3 point);
	std::vector<double> GetOptimal();
};

class Circen
{
public:
	double Accuracy;
	bool rebuild;
	std::vector<Value3> data;
	std::vector<double> Gbest;
public:
	Circen(double precision = 0.001);
	~Circen();
	void Setthread(ThreadPool &pool);
	void addPoint(Value3 point);
	std::vector<double> GetOptimal();
private:
	double precision;
	double distance(Value3 v1, Value3 v2);
	Value3 CalculaCenter(Value3 &point1,Value3 &point2,Value3 &point3,double &mid);
};

class FactorySolve
{
private:
	static GA* GASolver;
	static Ceres* CeresSolver;
	static Circen* CircenSolver;
	FactorySolve();
public:
	static GA* addSolve(Solve solve,int para_num,
		double dmin,double dmax,
		double(*_fun)(std::vector<double> &argv,std::vector<Value3> &data));
	static Ceres* addSolve(Solve solve);
	static Circen* addSolve(Solve solve,double precision);
	static void* getSolve(Solve solve);
	static void close();
	~FactorySolve();
};

#endif