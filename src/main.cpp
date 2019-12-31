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
static LinkList* iter[16]={0};
static unsigned char aim=0x00;
static uint8_t Smith=0xff;
float pi = 3.1415926;
//Size
float map_width = 40;
float map_length = 100;
//UAV
float UAV_filed = 11.54; //视野范围
const float UAV_low_filed = 7; // 下面相机视野，没有文件读取
float UAV_speed = 0.006; //速度
int BALLON_num = 6;
Value3 BALLON_Posion[6] = {0};
int AIM_num = 1;
float Simulate_speed = 0.002;

//绘图
auto draw_pic = [](RadioListen &radio,int msec){
	//100x60
	std::string curve = "SmithService";
	cvplot::setWindowTitle(curve,"curve");
	cvplot::moveWindow(curve, 0, 0);
	cvplot::resizeWindow(curve, 1200, 480);//*8 240*3 100x40
	auto &figure=cvplot::figure(curve);

	std::string PSO = "PSO";
	cvplot::setWindowTitle(PSO,"accuracy");
	cvplot::moveWindow(PSO, 1200, 0);
	cvplot::resizeWindow(PSO, 480, 480);
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
			{//待刺破气球数据
				// figure.series("filed").type(cvplot::Circle).color(cvplot::Yellow.alpha(192));
				for(int i=0;i<BALLON_num;i++){
					if(BALLON_Posion[i].Z!=0)//没掉
						BALLONdata.push_back(std::pair<float,float>(BALLON_Posion[i].X,BALLON_Posion[i].Y));
				}
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
				// printf("%f %f %f error: %f\n",pathparam[0],pathparam[1],pathparam[2],error.back().second);
				auto pathdata = PathGeneration3(pathparam[0],pathparam[1],pathparam[2]);
				Genpathdata.clear();
				for(auto &p:pathdata){
					// figure.series("Genpathdata").add(p.X,{p.Y,error.back().second } );
					Genpathdata.push_back(std::pair<float, float>(p.X,p.Y));
					// figure.series("Genpathdata").addValue(p.X,p.Y); 
				}
			}
#ifdef SIMULATE
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
#endif
			figure.series("AIM").set(AIMdata).type(cvplot::Dots).color(cvplot::Red);
			figure.series("UAV").set(UAVdata).type(cvplot::Dots).color(cvplot::Green);
#ifdef SIMULATE			
			//模拟器中实际AIM运动点
			figure.series("ActualAIMdata").type(cvplot::Dots).set(ActualAIMdata).color(cvplot::Black);
			//模拟器中实际AIM运动曲线
			figure.series("Actualpathdata").set(Actualpathdata).color(cvplot::Gray);
			//模拟器中气球位置
			figure.series("BALLON").set(BALLONdata).type(cvplot::Dots).color(cvplot::Red);
#endif
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
	uint SetlostNum[4]={0};
	const static int LimitLost = 2;//2.5s
	while(flag){
		if(!radio.CheckUAV(UAV1)){
			// std::cerr<<"UAV1 lost connect!"<<std::endl;
			if(SetlostNum[0]> LimitLost)
				uav[UAV1] = ROBOT_MODE_IN_INIT;
			else SetlostNum[0]++;
		}else{
			SetlostNum[0] = 0;
			uav[UAV1] = radio.GetUAVCommend(UAV1)|UAV1;
		}
		if(!radio.CheckUAV(UAV2)){
			// std::cerr<<"UAV2 lost connect!"<<std::endl;
			if(SetlostNum[1]> LimitLost)
				uav[UAV2] = ROBOT_MODE_IN_INIT;
			else SetlostNum[1]++;
		}else{
			SetlostNum[1] = 0;
			uav[UAV2] = radio.GetUAVCommend(UAV2)|UAV2;
		}
		if(!radio.CheckUAV(UAV3)){
			// std::cerr<<"UAV3 lost connect!"<<std::endl;
			if(SetlostNum[2]> LimitLost)
				uav[UAV3] = ROBOT_MODE_IN_INIT;
			else SetlostNum[2]++;
		}else{
			SetlostNum[2] = 0;
			uav[UAV3] = radio.GetUAVCommend(UAV3)|UAV3;
		}
		if(!radio.CheckUAV(AIM)){
			if(SetlostNum[3]> LimitLost)
				aim = ROBOT_MODE_IN_INIT|AIM;//丢失球
			else SetlostNum[3]++;
			// Smith = 0xff;
			for(int i=0x01;i<0x08;i=i<<1){
				if(uav[i]&ROBOT_MODE_IN_RETURN ){//有返回代表抓到了气球
					aim = ROBOT_MODE_IN_RETURN|AIM;//带着球返回了
					Smith =Marker(i);
					SetlostNum[3] = 0;
				}
			}
		}else{
			SetlostNum[3] = 0;
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
	BALLON_Posion[0] = config["ballon_point0"].as<Value3>();
	BALLON_Posion[1] = config["ballon_point1"].as<Value3>();
	BALLON_Posion[2] = config["ballon_point2"].as<Value3>();
	BALLON_Posion[3] = config["ballon_point3"].as<Value3>();
	BALLON_Posion[4] = config["ballon_point4"].as<Value3>();
	BALLON_Posion[5] = config["ballon_point5"].as<Value3>();
    ThreadPool pool(30);//建立线程池
	RadioListen radio_thread(pool,Listen_port);//监听串口
	pool.enqueue(time_chech_UAV, radio_thread,50);//周期确定消息 50ms
	//粒子群求解器
	FactorySolve::addSolve(GaSolve, 3, 5, map_length, CostPathGeneration3)->Setthread(pool);//加入求解器
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
			PathGeneration3(config["a"].as<float>(),
							config["b"].as<float>(),
							config["c"].as<float>()));

#endif
	pool.enqueue(draw_pic,radio_thread,50);//可视化
	//启动飞机 到一个点
	if(config["UAV1"]){//默认去刺球
		printf("start UAV1\n");
		UAV data(ROBOT_MODE_IN_INIT|UAV1);
		data.Posion = config["start_point1"].as<Value3>();
		radio_thread.SetUAVData(UAV1,data);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	if(config["UAV2"]){//默认去抓球
		printf("start UAV2\n");
		UAV data(ROBOT_MODE_IN_INIT|UAV2);
		data.Posion = config["start_point2"].as<Value3>();
		radio_thread.SetUAVData(UAV2,data);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	if(config["UAV3"]){//默认去刺球
		printf("start UAV3\n");
		UAV data(ROBOT_MODE_IN_INIT|UAV3);
		data.Posion = config["start_point3"].as<Value3>();
		radio_thread.SetUAVData(UAV3,data);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	time_t totalseconds = time(NULL);
	struct tm *st = localtime(&totalseconds);
	//服务端为多对一模式
	//      / UAV1
	//Smith - UAV2
	//      \ UAV3
	//三架飞机分别在自己的空域运动，地图大小为100x40，每块空域为33.33
	//UAV离地高度为5m，视野范围为6m~10m，因此按弓字形搜索
	// × × ×   ×  S   × x ×  x x x   x
	// ×   ×   ×  ×   ×   ×  x   x   x
	// ×   ×   ×  ×   ×   ×  x   x   x
	// ×   ×   ×  ×   ×   ×  x   x   x
	// ×   ×   ×  ×   ×   ×  x   x   x
	// ×   ×   ×  ×   ×   ×  x   x   x
	// ×   ×   ×  ×   ×   ×  x   x   x
	// ×   ×   ×  ×   ×   ×  ×   x   x
	// S   × × x  x x ×   ×  S   x x x
	// 
	// 起始UAV做扎气球运动，上方视野里有目标，切换到追气球模式，无人机可以
	// 接收到其他无人机的数据，但是收不到无人机之间相互发送的数据，若有一架追踪，
	// 另外两架继续进行扎气球任务
	//  - 若抓气球目标丢失，回到最近空域，若下降过程中视野里还有目标气球，继续上升进行抓取
	//  - 若气球目标丢失，回到最近空域，下降过程中没有目标气球，回到最近的空域，三个无人机
	//    继续做扎气球任务
	// 
	
	iter[UAV1] = Patharch(config["start_point1"].as<Value3>(),3,0.8);
	iter[UAV2] = Patharch(config["start_point2"].as<Value3>(),3,-0.8);
	iter[UAV3] = Patharch(config["start_point3"].as<Value3>(),3,0.8);
	while(1){
		S = aim;
		if(uav[UAV1]&0xf0)S = S | uav[UAV1];
		if(uav[UAV2]&0xf0)S = S | uav[UAV2];
		if(uav[UAV3]&0xf0)S = S | uav[UAV3];

		totalseconds = time(NULL);
		tm *st = localtime(&totalseconds);
		printf("[%02d:%02d:%02d]-> S: %02x AIM: %02x UAV1: %02x UAV2: %02x UAV3: %02x\n",
			st->tm_hour,st->tm_min,st->tm_sec,
			S,aim,uav[UAV1],uav[UAV2],uav[UAV3]);
		for(int i=0x01;i<0x08;i=i<<1){
			Marker now_uav = Marker(i);
			if(uav[now_uav]){//可通讯
				if((S&ROBOT_MODE_IN_CATCH) == ROBOT_MODE_IN_CATCH){
					if((uav[now_uav]&ROBOT_MODE_IN_CATCH) == ROBOT_MODE_IN_CATCH){
						//是当前飞机追气球，继续追
					}else{
						//不是当前飞机追气球，与另外 分成两个空域刺气球。
						UAV data((ROBOT_MODE_IN_MOVE)|now_uav);
						if(iter[now_uav]){
							data.Posion = iter[now_uav]->data;
							iter[now_uav] = iter[now_uav]->next;
							radio_thread.SetUAVData(now_uav,data);
						}
					}
				}else if((S&ROBOT_MODE_IN_STAB) == ROBOT_MODE_IN_STAB){//全都在刺球
					if(AIM_num){//还有目标机没有抓取 分配空域
						UAV data((ROBOT_MODE_IN_MOVE)|now_uav);
						if(iter[now_uav]){
							data.Posion = iter[now_uav]->data;
							iter[now_uav] = iter[now_uav]->next;
							radio_thread.SetUAVData(now_uav,data);
						}				
					}else{//没有目标后意味只有两架飞机,使用新的路线刺气球
						//重新分配空域
						if((uav[now_uav]&0xf0)!= ROBOT_MODE_IN_RETURN){
	
						}
					}
				}else if((S&ROBOT_MODE_IN_MOVE) == ROBOT_MODE_IN_MOVE){
					if(AIM_num||[](){for(auto &p:BALLON_Posion)if(p.Z!=0)return true;return false;}){//目标机或者气球没有刺
						//继续刺球，因为抓球优先级比刺高
						UAV data((ROBOT_MODE_IN_MOVE)|now_uav);
						if(iter[now_uav]){
							data.Posion = iter[now_uav]->data;
							iter[now_uav] = iter[now_uav]->next;
							radio_thread.SetUAVData(now_uav,data);
						}
					}else{//任务都完成了
						//指导返回
						UAV data((ROBOT_MODE_IN_RETURN)|now_uav);
						if(iter[now_uav]){
							data.Posion = Value3(0,0,0);
							radio_thread.SetUAVData(now_uav,data);
						}
					}
				}else if((S&ROBOT_MODE_IN_RETURN) == ROBOT_MODE_IN_RETURN){
					//指导返回
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(15));
			}else{
				//丢弃通讯
			}
		}

		if(aim&ROBOT_MODE_IN_CATCH){//记录球的位置  求解曲线 利用GA求解？
			//加入数据
		}else if(aim&ROBOT_MODE_IN_RETURN){
			AIM_num = 0;
			//分配空域
		}
		
		{//记录运行轨迹

		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
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