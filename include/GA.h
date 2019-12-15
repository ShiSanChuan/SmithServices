#ifndef _GA_H
#define _GA_H

#include "common.h"
#include "thread_pool.h"
#include <vector>
//特殊版
class GA

{
private:
	int param_size;
	float Accuracy;
	std::vector<float> paramter;
public:
	GA(std::vector<float> dmin,std::vector<float> dmax);
	void Setthread(ThreadPool &pool,GA *ga,int threadnum = 1);
	float GetAccuracy();
	std::vector<float> GetOptimal(float threshold);
	~GA();
};


#endif