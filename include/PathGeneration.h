#ifndef _PATHGENERATION_H
#define _PATHGENERATION_H
#include "common.h"
#include <vector>
std::vector<Value3> PathGeneration(float a=20.00,float b=20.00,float c=0.0, float d=0.0,float precision = 0.01);
std::vector<Value3> PathGeneration2(float x,float y,float r, float scale,float det,float precision=0.01);
std::vector<Value3> PathGeneration3(float r,float c,float d,float det=0,float precision=0.8);
LinkList *Pathline(Value3 start,int line_size,float precision=0.1);
LinkList * Patharch(Value3 start,int curve,float precision=0.1);
std::vector<Value3> BallonGeneration(int width,int length);
float  CostPathGeneration(std::vector<float> &argv,std::vector<Value3> &data);
float  CostPathGeneration2(std::vector<float> &argv,std::vector<Value3> &data);
float  CostPathGeneration3(std::vector<float> &argv,std::vector<Value3> &data);
float distance(Value3 v1,Value3 v2);
#endif