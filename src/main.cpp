#include <iostream>
#include "common.h"
#include "thread_pool.h"
#include "radio_listen_thread.h"
#include "UAVSimulate.h"
#include "timer.h"
#include "PathGeneration.h"
#include "cvplot.h"
int flag=true;
static unsigned char S=0x00;
static unsigned char uav[16]={0};
static unsigned char aim=0x00;
static uint8_t Smith=0xff;
static float distance(Value3 v1,Value3 v2){
	float dx = v1.X - v2.X;
	float dy = v1.Y - v2.Y;
	float dz = v1.Z - v2.Z;
	return sqrt(dx*dx+dy*dy+dz*dz);
}

auto draw_pic = [](RadioListen &radio,int msec){
	//100x60
	std::string name = "SmithService";
	cvplot::setWindowTitle(name,"origin curve");
	cvplot::moveWindow(name, 0, 0);
	cvplot::resizeWindow(name, 800, 480);//*8
	auto &figure=cvplot::figure(name);
	while(flag){
		figure.clear();
		{
			std::vector<std::pair<float, float> > UAVdata;
			std::vector<std::pair<float, float> > AIMdata;
			std::vector<std::pair<float, float> > BALLONdata;
			std::vector<std::pair<float, float> > Genpathdata;

			std::vector<std::pair<float, float> > Actualpathdata;
			std::vector<std::pair<float, float> > ActualAIMdata;
			{
				for(int i=0x01;i<0x08;i=i<<1){
					auto data = radio.GetUAVPosion(Marker(i));
					UAVdata.push_back(std::pair<float,float>(data.X,data.Y));
				}	
				UAVdata.push_back(std::pair<float, float>(800,480));
			}
			if((aim&0xf0) == ROBOT_MODE_IN_CATCH||(aim&0xf0) ==ROBOT_MODE_IN_RETURN){
				if(Smith!=0xff){
					auto data = radio.GetUAVPosion(Marker(Smith));
					AIMdata.push_back(std::pair<float, float>(data.X,data.Y));
				}else{
					auto data = radio.GetUAVPosion(AIM);
					AIMdata.push_back(std::pair<float, float>(data.X,data.Y));
				}
			}
			{
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
			figure.border(30).show();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(msec));
	}
};

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
    ThreadPool pool(30);
	RadioListen radio_thread(pool,"/dev/ttyUSB0");
	pool.enqueue(time_chech_UAV, radio_thread,50);//周期确定消息 50ms
#ifdef SIMULATE
	Simulate::init(pool,"/dev/ttyUSB1");//模拟器 UAV
	for(int i=0x01;i<0x08;i=i<<1){
		auto tmp = Simulate::addUAV(Marker(i));
		tmp->init(pool, &Simulate::Radio);
	}
	{
		auto tmp = Simulate::addUAV(AIM);
		tmp->init(pool, &Simulate::Radio,PathGeneration(200,200,400,240));
	}
#endif
	pool.enqueue(draw_pic,radio_thread,50);
	for(int i=0x01;i<0x08;i=i<<1){//命令启动
		UAV data(ROBOT_MODE_IN_INIT|i);
		data.Posion.X = rand()%800;
		data.Posion.Y = rand()%480;
		data.Posion.Z = 0;
		radio_thread.SetUAVData(Marker(i),data);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	while(1){

		std::this_thread::sleep_for(std::chrono::seconds(2));
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

    return 0;
}
//todo 8字模型？+ GA + 点适配进去
//模拟？
//显示 