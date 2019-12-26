#include <iostream>
#include <fstream>
#include "common.h"
#include "thread_pool.h"
#include "radio_listen_thread.h"
#include "UAVSimulate.h"
#include "timer.h"
#include "PathGeneration.h"
#include "cvplot.h"
#include "Solve.h"
int flag=true;
static unsigned char S=0x00;
static unsigned char uav[16]={0};
static unsigned char aim=0x00;
static uint8_t Smith=0xff;
float pi = 3.1415926;
//Size
float map_width = 40;
float map_length = 100;
//UAV
float UAV_filed = 11.54; //视野范围
float UAV_speed = 0.006; //速度
int BALLON_num = 6;
float Simulate_speed = 0.002;
std::string solve_name = "PSO";

//绘图
auto draw_pic = [](RadioListen &radio,int msec){
	//100x60
	std::string curve = "SmithService";
	cvplot::setWindowTitle(curve,"curve");
	cvplot::moveWindow(curve, 0, 0);
	cvplot::resizeWindow(curve, 1200, 720);//*8 240*3
	auto &figure=cvplot::figure(curve);

	std::string PSO = "PSO";
	cvplot::setWindowTitle(PSO,"accuracy");
	cvplot::moveWindow(PSO, 1200, 0);
	cvplot::resizeWindow(PSO, 360, 360);
	auto &PSOfigure=cvplot::figure(PSO);
	std::vector<std::pair<float, float> > error{std::make_pair(0, 20)};
	std::vector<std::pair<float, float> > Genpathdata;
	// Ceres* solve = (Ceres*)(FactorySolve::getSolve(CeresSolve));
	GA* solve = (GA*)(FactorySolve::getSolve(GaSolve));
	// Circen* solve = (Circen*)(FactorySolve::getSolve(CircenSolve));
	while(flag){
		figure.clear();
		{
			std::vector<std::pair<float, float> > UAVdata;
			std::vector<std::pair<float, float> > UAVfiled;
			std::vector<std::pair<float, float> > AIMdata;
			std::vector<std::pair<float, float> > BALLONdata;
			

			std::vector<std::pair<float, float> > Actualpathdata;
			std::vector<std::pair<float, float> > ActualAIMdata;
			{//UAV数据
			    figure.series("filed").type(cvplot::Circle).color(cvplot::Purple.alpha(192));
				for(int i=0x01;i<0x08;i=i<<1){
					auto data = radio.GetUAVPosion(Marker(i));
					UAVdata.push_back(std::pair<float,float>(data.X,data.Y));
					figure.series("filed").add(data.X,{data.Y,UAV_filed*4 } );
				}	
				UAVdata.push_back(std::pair<float, float>(map_length,map_width));

			}
			//模拟数据
			if((aim&0xf0) == ROBOT_MODE_IN_CATCH||(aim&0xf0) ==ROBOT_MODE_IN_RETURN){
				if(Smith!=0xff){
					auto data = radio.GetUAVPosion(Marker(Smith));
					AIMdata.push_back(std::pair<float, float>(data.X,data.Y));
				}else{
					auto data = radio.GetUAVPosion(AIM);
					AIMdata.push_back(std::pair<float, float>(data.X,data.Y));

					solve->addPoint(data);//添加数据点
					
				}
			}
			if(solve->Accuracy+1<20){//生成的估计路线
				error.push_back(std::make_pair(error.size(), solve->Accuracy));
				auto pathparam = solve->GetOptimal();
				printf("%f %f %f %f error: %f\n",pathparam[0],pathparam[1],pathparam[2],pathparam[3],
					error.back().second);
				auto pathdata = PathGeneration(pathparam[0],pathparam[1],pathparam[2],pathparam[3],pathparam[4]);
				// auto pathdata = PathGeneration(pathparam[0],pathparam[1],pathparam[2],pathparam[3]);
				Genpathdata.clear();
				for(auto &p:pathdata){
					// figure.series("Genpathdata").add(p.X,{p.Y,error.back().second } );
					Genpathdata.push_back(std::pair<float, float>(p.X,p.Y));
					// figure.series("Genpathdata").addValue(p.X,p.Y); 
				}
			}
			{//模拟的寻找飞机数据
				UAVSimulate *aim  = Simulate::getUAV(AIM);
				for(auto &p:aim->path){
					Actualpathdata.push_back(std::pair<float, float>(p.X,p.Y));
				}
			}
			if(Smith==0xff){
				UAVSimulate *aim  = Simulate::getUAV(AIM);
				ActualAIMdata.push_back(std::pair<float, float>(aim->uav.Posion.X,aim->uav.Posion.Y));
			}
			figure.series("AIM").set(AIMdata).type(cvplot::Dots).color(cvplot::Red);
			figure.series("UAV").set(UAVdata).type(cvplot::Dots).color(cvplot::Green);
			figure.series("ActualAIMdata").type(cvplot::Dots).set(ActualAIMdata).color(cvplot::Black);
			
			figure.series("Actualpathdata").set(Actualpathdata).color(cvplot::Gray);
			figure.series("Genpathdata").set(Genpathdata).color(cvplot::Red);
    
			figure.border(30).show();
		}
		PSOfigure.clear();

		{
			PSOfigure.series("error").set(error).color(cvplot::Orange);
			PSOfigure.border(30).show();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(msec));
	}
};
//定时检测
auto time_chech_UAV = [](RadioListen &radio,int msec){
	while(flag){
		if(!radio.CheckUAV(UAV1)){
			// std::cerr<<"UAV1 lost connect!"<<std::endl;
			uav[UAV1] = ROBOT_MODE_IN_INIT;
		}else{
			uav[UAV1] = radio.GetUAVCommend(UAV1)|UAV1;
		}
		if(!radio.CheckUAV(UAV2)){
			// std::cerr<<"UAV2 lost connect!"<<std::endl;
			uav[UAV2] = ROBOT_MODE_IN_INIT;
		}else{
			uav[UAV2] = radio.GetUAVCommend(UAV2)|UAV2;
		}
		if(!radio.CheckUAV(UAV3)){
			// std::cerr<<"UAV3 lost connect!"<<std::endl;
			uav[UAV3] = ROBOT_MODE_IN_INIT;
		}else{
			uav[UAV3] = radio.GetUAVCommend(UAV3)|UAV3;
		}
		if(!radio.CheckUAV(AIM)){
			aim = ROBOT_MODE_IN_INIT|AIM;//丢失球
			// Smith = 0xff;
			for(int i=0x01;i<0x08;i=i<<1){
				if(uav[i]&ROBOT_MODE_IN_RETURN ){//有返回代表抓到了气球
					aim = ROBOT_MODE_IN_RETURN|AIM;//带着球返回了
					Smith =Marker(i);
				}
			}
		}else{
			aim = ROBOT_MODE_IN_CATCH|AIM;//找到球
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(msec));
	}
	flag =false;
};

int main(int argc, const char** argv)
{
	srand(time(NULL));

	if(argc<2){
		std::cout<<"./exe yaml_path";
		return 0;
	}
	//传递参数
	std::string yamlpath(argv[1]);
	YAML::Node config = YAML::LoadFile(yamlpath);
	std::string Listen_port = config["Listen_port"].as<std::string>();
	std::string Simulate_port = config["Simulate_port"].as<std::string>();
	map_width = config["map_width"].as<float>();
	map_length = config["map_length"].as<float>();
	UAV_filed = config["UAV_filed"].as<float>(); //视野范围
	UAV_speed = config["UAV_speed"].as<float>(); //速度
	BALLON_num = config["BALLON_num"].as<float>();
	Simulate_speed = config["Simulate_speed"].as<float>();
    ThreadPool pool(30);//建立线程池
	RadioListen radio_thread(pool,Listen_port);//监听串口
	pool.enqueue(time_chech_UAV, radio_thread,50);//周期确定消息 50ms
	//粒子群求解器
	FactorySolve::addSolve(GaSolve, 4, 5, map_length, CostPathGeneration)->Setthread(pool);//加入求解器
	//谷歌求解器
	FactorySolve::addSolve(CeresSolve)->Setthread(pool);
	//圆求解器
	FactorySolve::addSolve(CircenSolve,0.001)->Setthread(pool);

#ifdef SIMULATE
	Simulate::init(pool,Simulate_port);//模拟器 UAV
	if(config["UAV1"])
		Simulate::addUAV(UAV1)->init(pool,&Simulate::Radio,config["start_point1"].as<Value3>());
	if(config["UAV2"])
		Simulate::addUAV(UAV2)->init(pool,&Simulate::Radio,config["start_point2"].as<Value3>());
	if(config["UAV3"])
		Simulate::addUAV(UAV3)->init(pool,&Simulate::Radio,config["start_point3"].as<Value3>());
	if(config["Simulate_AIM"])
		Simulate::addUAV(AIM)->init(pool, &Simulate::Radio,
			PathGeneration(config["a"].as<float>(),
							config["b"].as<float>(),
							config["c"].as<float>(),
							config["d"].as<float>()));

#endif
	pool.enqueue(draw_pic,radio_thread,50);//可视化
	//启动飞机
	if(config["UAV1"]){
		UAV data(ROBOT_MODE_IN_INIT|UAV1);
		data.Posion = config["first_point1"].as<Value3>();
		radio_thread.SetUAVData(UAV1,data);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	if(config["UAV2"]){
		UAV data(ROBOT_MODE_IN_INIT|UAV2);
		data.Posion = config["first_point2"].as<Value3>();
		radio_thread.SetUAVData(UAV2,data);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	if(config["UAV3"]){
		UAV data(ROBOT_MODE_IN_INIT|UAV3);
		data.Posion = config["first_point3"].as<Value3>();
		radio_thread.SetUAVData(UAV3,data);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	while(1){
		//未完成
		std::this_thread::sleep_for(std::chrono::seconds(2));
		printf("aim: %x UAV1: %x UAV2 %x UAV3 %x\n",aim,uav[UAV1],uav[UAV2],uav[UAV3] );
	}
	return 0;
	while(1){
		S = aim;
		if(uav[UAV1]&0xf0)S = S | uav[UAV1];
		if(uav[UAV2]&0xf0)S = S | uav[UAV2];
		if(uav[UAV3]&0xf0)S = S | uav[UAV3];
		if((S&0xf0)){
			if((S&ROBOT_MODE_IN_CATCH) == ROBOT_MODE_IN_CATCH){
				//抓气球 其中一个是找到了球
				float min_distance = UINT32_MAX;
				float uav_distance ;
				Marker index ;
				Value3 aim_status = radio_thread.GetUAVPosion(AIM);
				for(int i=0x01;i<0x80;i=i<<1){
					if(S&i){
						uav_distance = distance(radio_thread.GetUAVPosion(Marker(i)),aim_status);
						min_distance = std::min(min_distance,uav_distance);
						if(min_distance == uav_distance)index = Marker(i);
					}
				}
				if((uav[index]&ROBOT_MODE_IN_CATCH) == ROBOT_MODE_IN_CATCH){
					//nothing todo 当前飞机继续追
					// radio_thread.SetUAVCommend(index,uav[index]);
				}else {
					//最近的飞机不是追的飞机,没有追的飞机需要闪避(飞机遇到自己的飞机)
					UAV data;
					data.situation = ROBOT_MODE_IN_MOVE;
					data.Posion = radio_thread.GetUAVPosion(index);
					data.Posion.X = data.Posion.X - (aim_status.X - data.Posion.X)/100;//偏离一点
					data.Posion.Y = data.Posion.Y - (aim_status.Y - data.Posion.Y)/100;
					radio_thread.SetUAVData(index,data);
				}
				for(int i=0x01;i<0x80;i++){
					if((i!=index)&&(S&i)){
						if((uav[i]&ROBOT_MODE_IN_CATCH) ==  ROBOT_MODE_IN_CATCH){//不能有两架同事追球
							//做其他事
							UAV data;
							data.situation = ROBOT_MODE_IN_MOVE;
							data.Posion = radio_thread.GetUAVPosion(Marker(i));
							data.Posion.X = data.Posion.X - (aim_status.X - data.Posion.X)/100;//偏离一点
							data.Posion.Y = data.Posion.Y - (aim_status.Y - data.Posion.Y)/100;
							radio_thread.SetUAVData(index,data);							
						}else if((uav[i]&ROBOT_MODE_IN_STAB) ==  ROBOT_MODE_IN_STAB){
							//另外刺气球 正常
						}else if((uav[i]&ROBOT_MODE_IN_MOVE) ==  ROBOT_MODE_IN_MOVE){
							//去刺气球
						}else if((uav[i]&ROBOT_MODE_IN_RETURN) == ROBOT_MODE_IN_RETURN){
							//当前UAV完成任务了
						}
					}
				}
			}else if((S&ROBOT_MODE_IN_STAB) == ROBOT_MODE_IN_STAB){//刺气球
				for(int i=0x01;i<0x80;i=i<<1){
					if((S&i)&&(uav[i]&ROBOT_MODE_IN_STAB)){
						//仍然在刺气球
						if((aim&ROBOT_MODE_IN_INIT)){//球丢了或者是没找到球
							//估计刺球 可能 或者去找球
							Value3 aim_status = radio_thread.GetUAVPosion(AIM);//最后球丢失的地点

						}else {//任然刺气球哦

						}
					}else if((S&i)&&(uav[i]&ROBOT_MODE_IN_MOVE)){
						//找气球可能点哦
						if((aim&ROBOT_MODE_IN_INIT)){//球丢了或者是没找到球
							//直接找球
						}else{
							//去另外刺气球
						}
					}//飞机有返回的话是找到了球
				}
			}else if((S&ROBOT_MODE_IN_MOVE) == ROBOT_MODE_IN_MOVE){//移动 依据8字模型建立

			}else if((S&ROBOT_MODE_IN_RETURN) == ROBOT_MODE_IN_RETURN){//返回

			}
		}else {//初始化启动
			//ROBOT_MODE_IN_INIT
		}
		if(aim&ROBOT_MODE_IN_CATCH){//记录球的位置  求解曲线 利用GA求解？


		}

		//逻辑仲裁
		//
		//
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
#ifdef SIMULATE
	Simulate::close();
#endif
	FactorySolve::close();
    return 0;
}
//todo 8字模型？+ GA + 点适配进去
//模拟？
//显示 