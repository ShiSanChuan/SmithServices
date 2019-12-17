#include "GA.h"
#include "PathGeneration.h"
#include <mutex>
std::map<std::string,GA*> FactoryGA::GAsolve;
static std::mutex mut;
void run(GA *pso){
	bool start=false;
	while(flag){
		if(pso->data.size()){
			if(!start){//最初有一次最优解
				pso->init();
				start = true;
			}else{
				std::lock_guard<std::mutex> lock(mut);
				pso->ranking();
				pso->update(0);
			}
		}
		if(!start)
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		else
			std::this_thread::sleep_for(std::chrono::milliseconds(10));	
	}
	flag = false;
}

GA::GA(int para_num,float dmin,float dmax,
	float(*_fun)(std::vector<float> &argv,std::vector<Value3> &data)){
	fun = _fun;
	this->dmin = dmin;
	this->dmax = dmax;
	chrom_num = 500;
	c1 = 2;
	c2 = 2;
	k = 1;
	w = 0.9;
	Accuracy = INT16_MAX;
	this->para_num = para_num;
	speed = cv::Mat(cv::Size(para_num,chrom_num),CV_32FC1,cv::Scalar(0));
	Population = cv::Mat(cv::Size(para_num,chrom_num),CV_32FC1,cv::Scalar(0));
	ost=std::vector<float>(chrom_num,0);
	post=std::vector<float>(chrom_num,0);
	cv::RNG rng(time(NULL));
	// rng.fill(Population, cv::RNG::NORMAL,(dmax-dmin)/2 + dmin , 50 );//接近高斯分布
	rng.fill(Population, cv::RNG::UNIFORM,dmin ,dmax );
	for(int i=0;i<chrom_num;i++){
		Population.at<float>(i,0) = (int)Population.at<float>(i,0)%(int)(map_width*2);
		Population.at<float>(i,1) = (int)Population.at<float>(i,1)%(int)(map_width*2);
		Population.at<float>(i,2) = (int)Population.at<float>(i,2)%(int)map_width;
		Population.at<float>(i,3) = (int)Population.at<float>(i,3)%(int)map_length;
	}
	rng.fill(speed, cv::RNG::UNIFORM,0 , 0.5);//速度不是
	Population.copyTo(Pbest);

}
GA::~GA(){

}
void GA::Setthread(ThreadPool &pool){
	pool.enqueue(run, this);
}
std::vector<float> GA::GetOptimal(){
	return Gbest;
}
void GA::addPoint(Value3 point){
	std::lock_guard<std::mutex> lock(mut);
	if(data.size()&&(distance(data.back(),point)>0.01)){
		data.push_back(point);
	}else if(!data.size()){
		data.push_back(point);
		for(int i=0;i<chrom_num;i+=1){
			Population.at<float>(i,0) = (int)(rand()%(int)((dmax-dmin)+dmin))%(int)(map_width*2);
			Population.at<float>(i,1) = (int)(rand()%(int)((dmax-dmin)+dmin))%(int)(map_width*2);
			Population.at<float>(i,2) = (int)(rand()%(int)((dmax-dmin)+dmin))%(int)map_width;
			Population.at<float>(i,3) = (int)(rand()%(int)((dmax-dmin)+dmin))%(int)map_length;
		}
		w=0.9;
	}
}
void GA::init(){
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
	int gbb=fun(Gbest,data);
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

GA* FactoryGA::addSolve(std::string name,int para_num,
	float dmin,float dmax,
	float(*_fun)(std::vector<float> &argv,std::vector<Value3> &data)){
	if(GAsolve.count(name)){
		return GAsolve[name];
	}else{
		GAsolve[name] = new GA(para_num,dmin,dmax,_fun);
		return GAsolve[name];
	}
	return nullptr;
}
GA* FactoryGA::getSolve(std::string name){
	if(GAsolve.count(name)){
		return GAsolve[name];
	}
	return nullptr;
}
void FactoryGA::close(){
	for(auto iter=GAsolve.begin();iter!=GAsolve.end();iter++){
		GA* p = iter->second;
		delete  p;
		iter->second = nullptr;
	}
}