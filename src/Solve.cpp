#include "Solve.h"
#include "PathGeneration.h"
#include <mutex>

Ceres* FactorySolve::CeresSolver;
GA* FactorySolve::GASolver;
Circen* FactorySolve::CircenSolver;

GA* FactorySolve::addSolve(Solve solve,int para_num,
	float dmin,float dmax,
	float(*_fun)(std::vector<float> &argv,std::vector<Value3> &data)){

	if(GASolver!=nullptr){
		return GASolver;
	}else{
		GASolver = new GA(para_num,dmin,dmax,_fun);
		return GASolver;
	}
	return nullptr;
}
Ceres* FactorySolve::addSolve(Solve solve){
	if(CeresSolver!=nullptr){
		return CeresSolver;
	}else{
		CeresSolver = new Ceres;
		return CeresSolver;
	}
	return nullptr;
}
Circen* FactorySolve::addSolve(Solve solve,float precision){
	if(CircenSolver!=nullptr){
		return CircenSolver;
	}else{
		CircenSolver = new Circen(precision);
		return CircenSolver;
	}
	return nullptr;
}
void* FactorySolve::getSolve(Solve solve){
	switch(solve){
		case GaSolve:{
			if(GASolver)return GASolver;
			else return nullptr;
			break;
		}
		case CeresSolve:{
			if(CeresSolver)return CeresSolver;
			else return nullptr;
		}
		case CircenSolve:{
			if(CircenSolver)return CircenSolver;
			else return nullptr;
		}
	}
	return nullptr;
}
void FactorySolve::close(){
	if(GASolver)delete GASolver;
	if(CeresSolver)delete CeresSolver;
	if(CircenSolver)delete CircenSolver;
}