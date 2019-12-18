#include "Solve.h"
#include "PathGeneration.h"
#include <mutex>
GA* FactorySolve::GASolver;
Ceres* FactorySolve::CeresSolver;
static std::mutex mut;
void runPSO(GA *pso){
	while(flag){
		if(pso->data.size()){
			std::lock_guard<std::mutex> lock(mut);
			pso->ranking();
			pso->update(0);
			if(pso->Accuracy+1>INT16_MAX){
				printf("rebuild\n");
				pso->init();
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));	
	}
	flag = false;
}
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
GA::GA(int para_num,float dmin,float dmax,
	float(*_fun)(std::vector<float> &argv,std::vector<Value3> &data)){
	fun = _fun;
	this->dmin = dmin;
	this->dmax = dmax;
	chrom_num = 800;
	c1 = 2;
	c2 = 2;
	k = 1;
	w = 0.9;
	Accuracy = INT16_MAX;
	min_Accuracy = INT16_MAX;
	this->para_num = para_num;
	speed = cv::Mat(cv::Size(para_num,chrom_num),CV_32FC1,cv::Scalar(0));
	Population = cv::Mat(cv::Size(para_num,chrom_num),CV_32FC1,cv::Scalar(0));
	ost=std::vector<float>(chrom_num,0);
	post=std::vector<float>(chrom_num,0);
}
GA::~GA(){

}
void GA::Setthread(ThreadPool &pool){
	pool.enqueue(runPSO, this);
}
std::vector<float> GA::GetOptimal(){
	return Gbest;
}
void GA::addPoint(Value3 point){
	std::lock_guard<std::mutex> lock(mut);
	if(data.size()&&(distance(data.back(),point)>1)){
		data.push_back(point);
		printf("%f %f\n",fun(Gbest,data),Accuracy );
		if(fun(Gbest,data)>Accuracy){//新数据 模型不满足重排
			init();
			w=0.9;
		}
	}else if(!data.size()){
		data.push_back(point);
		init();
	}
}
void GA::init(){
	cv::RNG rng(time(NULL));
	rng.fill(Population, cv::RNG::UNIFORM,dmin ,dmax );
	rng.fill(speed, cv::RNG::UNIFORM,0 , 0.5);//速度不是
	Population.copyTo(Pbest);

	for(int i=0,min=0;i<chrom_num;i++){
		std::vector<float> argv;
		for(int j=0;j<para_num;j++)
			argv.push_back(Population.at<float>(i,j));
		int result=fun(argv,data);
		if(i==0||result<min){
			Gbest=argv;
			min=result;
		}
		ost[i]=(result);
	}
}
void GA::ranking(){
	Accuracy = fun(Gbest,data);
	for(int i=0;i<chrom_num;i++){
		std::vector<float> argv;
		for(int j=0;j<para_num;j++){
			//更新速度 updata speed
			speed.at<float>(i,j)=(float)(w*speed.at<float>(i,j)+
				c1*(Gbest[j]-Population.at<float>(i,j))*(float)(rand()%1000)/1000+
				c2*(Pbest.at<float>(i,j)-Population.at<float>(i,j))*(float)(rand()%1000)/1000);
			//更新位置 update site
			Population.at<float>(i,j)=(float)Population.at<float>(i,j)+k*(float)(speed.at<float>(i,j));
			if((float)Population.at<float>(i,j)>dmax)
				Population.at<float>(i,j)=dmax;
			if((float)Population.at<float>(i,j)<dmin)
				Population.at<float>(i,j)=dmin;
			argv.push_back(Population.at<float>(i,j));
		}
		post[i]=fun(argv,data);
	}
}
void GA::update(bool para){
	double gbb=fun(Gbest,data);
	Accuracy = gbb;
	for(int i=0;i<chrom_num;i++){
		if((ost[i]>post[i])^para){//若参数小于局部最优，参数被局部数据取代
			for(int j=0;j<para_num;j++)
				Pbest.at<float>(i,j)=(float)Population.at<float>(i,j);
			ost[i]=post[i];
		}
		if((post[i]<gbb)^para){//比全局最优
			for(int j=0;j<para_num;j++)
				Gbest[j]=(float)Population.at<float>(i,j);
			gbb=post[i];
		}
	}
	w = w - (std::exp(w-0.4)-1)/2;
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
	Accuracy = INT16_MAX;
	data.push_back(point);
	rebuild = true;
}
std::vector<float> Ceres::GetOptimal(){
	std::lock_guard<std::mutex> lock(mut);
	return Gbest;
}

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
		CeresSolver = new Ceres();
		return CeresSolver;
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
	}
	return nullptr;
}
void FactorySolve::close(){
	if(GASolver)delete GASolver;
	if(CeresSolver)delete CeresSolver;
}