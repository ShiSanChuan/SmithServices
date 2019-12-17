#ifndef _GA_H
#define _GA_H

#include "common.h"
#include "thread_pool.h"
#include "opencv2/opencv.hpp"
#include <vector>
#include <functional>
#include <map>
//特殊版
class GA

{
public:
	float Accuracy;
	std::vector<Value3> data;
private:
	int chrom_num;
	int para_num;
	float c1;
	float c2;
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
class FactoryGA
{
private:
	static std::map<std::string,GA*> GAsolve;
	FactoryGA();
public:
	static GA* addSolve(std::string name,int para_num,
		float dmin,float dmax,
		float(*_fun)(std::vector<float> &argv,std::vector<Value3> &data));
	static GA* getSolve(std::string name);
	static void close();
	~FactoryGA();
};

#endif