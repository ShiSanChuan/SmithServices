#ifndef _PATHGENERATION_H
#define _PATHGENERATION_H
#include "common.h"
#include <vector>
std::vector<Value3> PathGeneration(double a=20.00,double b=20.00,double c=0.0, double d=0.0,double precision = 0.01);
std::vector<Value3> PathGeneration2(double x,double y,double r, double scale,double det,double precision=0.01);
std::vector<Value3> PathGeneration3(double r,double c,double d,double det=0,double precision=0.8);
LinkList *Pathline(Value3 start,int line_size,double precision=0.1);
LinkList * Patharch(Value3 start,int curve,double precision=0.1);
std::vector<Value3> BallonGeneration(int width,int length);
double  CostPathGeneration(std::vector<double> &argv,std::vector<Value3> &data);
double  CostPathGeneration2(std::vector<double> &argv,std::vector<Value3> &data);
double  CostPathGeneration3(std::vector<double> &argv,std::vector<Value3> &data);
double distance(Value3 v1,Value3 v2);
#endif