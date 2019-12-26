#include "Solve.h"
#include "PathGeneration.h"
#include <mutex>

static std::mutex mut;
void runCeres(Ceres *c){
	while(flag){
		if(c->rebuild){
			double param[4] = {20,20,map_length/2,map_width/2};
			ceres::Problem problem;
			for(int i=0;i<c->data.size();i++){
			    ceres::CostFunction *cost_function = new AnalyticCostFunctor(c->data[i].X, c->data[i].Y);
			    problem.AddResidualBlock(cost_function, NULL, param);
			}
			ceres::Solver::Options options;
			options.linear_solver_type = ceres::DENSE_QR;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			c->Accuracy = summary.final_cost;
			printf("error : %lf\n",summary.final_cost);
			{
				std::lock_guard<std::mutex> lock(mut);
				c->Gbest[0] = param[0];
				c->Gbest[1] = param[1];
				c->Gbest[2] = param[2];
				c->Gbest[3] = param[3];
			}
			// for(int i=0;i<c->data.size();i++){
			// 	printf("%f,%f,",c->data[i].X,c->data[i].Y );
			// }
			// printf("\n");
			printf("->  %f %f %f %f\n",param[0],param[1],param[2],param[3] );
			c->rebuild = false;
		}else
		std::this_thread::sleep_for(std::chrono::milliseconds(100));	
		
	}
	flag = false;
}
Ceres::Ceres(){
	Accuracy = INT16_MAX;
	rebuild = false;
	
	Gbest = std::vector<float> {map_width,map_width,map_length/2,map_width/2};	
}
Ceres::~Ceres(){

}
void Ceres::Setthread(ThreadPool &pool){
	pool.enqueue(runCeres, this);
}
void Ceres::addPoint(Value3 point){
	if(data.size()&&(distance(data.back(),point)>1)){
		Accuracy = INT16_MAX;
		data.push_back(point);
		rebuild = true;
	}else if(!data.size()){
		Accuracy = INT16_MAX;
		data.push_back(point);
		rebuild = true;
	}
}
std::vector<float> Ceres::GetOptimal(){
	std::lock_guard<std::mutex> lock(mut);
	return Gbest;
}