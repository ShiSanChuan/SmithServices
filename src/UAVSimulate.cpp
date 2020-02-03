#include "UAVSimulate.h"
#include "PathGeneration.h"
#include <random>
#include <fstream>
//模拟一个UAV飞行哦
//
//
RadioInterface Simulate::Radio[UAV_size];
static std::mutex rmut;
static std::mutex wmut;
UAVSimulate* Simulate::Worker[Device_size];

static void Workrun(RadioInterface *Radio,UAVSimulate** Worker){
    Value3 data;
    RadioPacket recvPacket;
	while(flag){
		if(Radio->dataRecv(recvPacket)==0){
			unsigned char CMD = recvPacket.getCMD();
			std::lock_guard<std::mutex> lock(rmut);
			unsigned char device = recvPacket.getUncharInBuffer(2);
			if(device!=AIM&&Worker[device]){
				data.X = recvPacket.getFloatInBuffer(3);
				data.Y = recvPacket.getFloatInBuffer(7);
				data.Z = recvPacket.getFloatInBuffer(11);
				if(Worker[device]->SetSituation(CMD))
					Worker[device]->SetPosion(data);//设定目标
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
	flag = false;
}

void run(RadioInterface *Radio,UAVSimulate *Suav){
	double speed = UAV_speed;//飞行速度 km/h -> 1/360 m/ms 
	double field = UAV_filed;	 // 可以看见球的范围
	Value3 backaim;
	bool return_flag = false;
	uint8_t ID = Suav->uav.ID;
	uint8_t situation = Suav->uav.situation;
	Value3 AIM_Position;
	Value3 NOW_Position;
	Value3 *BALL_p=nullptr;
	for(int i=0;i<UAV_size;i++){//该线程飞机标示
		if(Simulate::getUAV(Marker(i))==Suav){
			ID = i;
			Suav->uav.ID = i;
			break;
		}
	}
	printf("ID %d situation %01x\n", ID,situation);
	LinkList *Line = nullptr;//运动一次后停止
	LinkList *Arch = nullptr;
	while(flag){
		if(situation!=Suav->uav.situation){//改变状态 抓取和刺球没有权限
				situation = Suav->uav.situation;
			switch(situation){
				case ROBOT_MODE_IN_INIT:{//初始化
					printf("[control model mode ]: %d Switch to INIT Mode!\n",ID);
					std::this_thread::sleep_for(std::chrono::seconds(1));
					NOW_Position = Value3(0,0,0);
					printf("[control model mode ]: %d INIT Mode IS OK!\n",ID);
					break;
				}
				case ROBOT_MODE_IN_TAKEOFF:{//自动起飞
					printf("[control model mode ]: %d Switch to TAKEOFF Mode!\n",ID);
					std::this_thread::sleep_for(std::chrono::seconds(2));
					NOW_Position = Value3(0,0,5);
					printf("[control model mode ]: %d TAKEOFF Mode IS OK!\n",ID);
					break;
				}
				case ROBOT_MODE_IN_MOVETO:{//移动到一个点
					printf("[control model mode ]: %d Switch to MOVETO Mode!\n",ID);
					AIM_Position = Suav->uav.Posion;
					printf("[control model mode ]: %d MOVETO Mode IS OK!\n",ID);
					break;
				}
				case ROBOT_MODE_IN_LINE:{//线型移动
					printf("[control model mode ]: %d Switch to LINE Mode!\n",ID);
					if(Line==nullptr){
						Line = Pathline(Value3(Suav->uav.Posion.X,Suav->uav.Posion.Y,5),Suav->uav.Posion.Z,0.8);
					}
					{//test
						std::fstream file("linepath_"+std::to_string(Suav->uav.ID)+".csv",std::ios::out);
						LinkList * N = Line;
						file<<"X\tY\tZ\n";
						while(N!=nullptr&&N->next!=Line){
							file<<N->data.X<<"\t"<<N->data.Y<<"\t"<<N->data.Z<<"\n";
							N=N->next;
						}
						file.close();
					}
					printf("[control model mode ]: %d LINE Mode IS OK!\n",ID);
					break;
				}
				case ROBOT_MODE_IN_ARCH:{
					printf("[control model mode ]: %d Switch to STAB Mode!\n",ID);
					if(Arch==nullptr)
						Arch = Patharch(Value3(Suav->uav.Posion.X,Suav->uav.Posion.Y,5),Suav->uav.Posion.Z,0.8);
					{//test
						std::fstream file("archpath_"+std::to_string(Suav->uav.ID)+".csv",std::ios::out);
						LinkList * N = Arch;
						file<<"X\tY\tZ\n";
						while(N!=nullptr&&N->next!=Arch){
							file<<N->data.X<<"\t"<<N->data.Y<<"\t"<<N->data.Z<<"\n";
							N=N->next;
						}
						file.close();
					}
					printf("[control model mode ]: %d STAB Mode IS OK!\n",ID);
					break;
				}
				case ROBOT_MODE_IN_CATCH:{//抓取气球
					printf("[control model mode ]: %d Switch to CATCH Mode!\n",ID);
					
					printf("[control model mode ]: %d CATCH Mode IS OK!\n",ID);
					break;
				}
				case ROBOT_MODE_IN_STAB:{//刺气球
					printf("[control model mode ]: %d Switch to STAB Mode!\n",ID);
					// AIM = Suav->uav.Posion;
					printf("[control model mode ]: %d STAB Mode IS OK!\n",ID);
					break;
				}
				case ROBOT_MODE_IN_RETURN:{//返回
					printf("[control model mode ]: %d Switch to RETURN Mode!\n",ID);
					AIM_Position = Value3(0,0,0);
					printf("[control model mode ]: %d RETURN Mode IS OK!\n",ID);
					break;
				}
				case ROBOT_MODE_IN_EMPTY:{//空任务
					printf("[control model mode ]: %d Switch to EMPTY Mode!\n",ID);

					printf("[control model mode ]: %d EMPTY Mode IS OK!\n",ID);
					break;
				}
			}
		}
		switch(situation){//动作
			case ROBOT_MODE_IN_LINE:{//线型移动
				{
					UAVSimulate* aim = Simulate::getUAV(AIM);
					double dis = distance(aim->uav.Posion,NOW_Position);
					if(dis<std::abs(aim->uav.Posion.Z - NOW_Position.Z)*1.2){//视野里
						Suav->uav.situation = ROBOT_MODE_IN_CATCH;
					}
				}
				if(situation == ROBOT_MODE_IN_LINE){
					double dis = distance(NOW_Position,Line->data);
					AIM_Position = Line->data;
					while(dis<1.6){
						Line = Line->next;
						dis = distance(NOW_Position,Line->data);
					}
					//移动
					double radio = (speed*50)/dis;
					NOW_Position.X = NOW_Position.X + (Line->data.X - NOW_Position.X)*radio;
					NOW_Position.Y = NOW_Position.Y + (Line->data.Y - NOW_Position.Y)*radio;
					NOW_Position.Z = NOW_Position.Z + (Line->data.Z - NOW_Position.Z)*radio*0.3;
					if(Line->next == nullptr){
						Suav->uav.situation = ROBOT_MODE_IN_RETURN;
					}
				}
				break;
			}
			case ROBOT_MODE_IN_ARCH:{
				for(int i=0;i<BALLON_num;i++){
					if(BALLON_Posion[i].Z==0)continue;//已扎到
					double dis = distance(NOW_Position,BALLON_Posion[i]);
					if(dis<UAV_low_filed){
						BALL_p = nullptr;
						BALL_p = &BALLON_Posion[i];
						Suav->uav.situation = ROBOT_MODE_IN_STAB;
						break;
					}
				}
				if(situation == ROBOT_MODE_IN_ARCH){
					double dis = distance(Arch->data,NOW_Position);
					if(dis<1.6){
						Arch = Arch->next;
						dis = distance(Arch->data,NOW_Position);
					}
					double radio = (speed*50)/dis;
					NOW_Position.X = NOW_Position.X + (Arch->data.X - NOW_Position.X)*radio;
					NOW_Position.Y = NOW_Position.Y + (Arch->data.Y - NOW_Position.Y)*radio;
					NOW_Position.Z = NOW_Position.Z + (Arch->data.Z - NOW_Position.Z)*radio*0.3;
				}
				if(Arch->next == nullptr){
					Suav->uav.situation = ROBOT_MODE_IN_RETURN;
				}
				break;
			}
			case ROBOT_MODE_IN_CATCH:{//抓取气球
				UAVSimulate* aim = Simulate::getUAV(AIM);
				if(std::abs(aim->uav.Posion.X - NOW_Position.X)<0.5 &&
					std::abs(aim->uav.Posion.Y - NOW_Position.Y)<0.5 ){
					Suav->uav.situation = ROBOT_MODE_IN_RETURN;
				}else{
					double dis = distance(aim->uav.Posion,NOW_Position);
					double radio = (speed*50)/dis;
					NOW_Position.X = NOW_Position.X + (aim->uav.Posion.X - NOW_Position.X)*radio;
					NOW_Position.Y = NOW_Position.Y + (aim->uav.Posion.Y - NOW_Position.Y)*radio;
					NOW_Position.Z = NOW_Position.Z + (aim->uav.Posion.Z - NOW_Position.Z)*radio*0.3;
					Simulate::SendPack(situation,aim->uav.ID,aim->uav.Posion);
					std::this_thread::sleep_for(std::chrono::milliseconds(15));
				}
				break;
			}
			case ROBOT_MODE_IN_STAB:{//刺气球
				double dis = distance(*BALL_p,NOW_Position);
				if(dis<speed*50){
					assert(BALL_p!=nullptr);
					BALL_p->Z = 0;//已经被刺过	
					BALL_p = nullptr;	
					Suav->uav.situation = ROBOT_MODE_IN_ARCH;
				}else{
					assert(BALL_p!=nullptr);
					double radio = (speed*50)/dis;
					NOW_Position.X = NOW_Position.X + (BALL_p->X - NOW_Position.X)*radio;
					NOW_Position.Y = NOW_Position.Y + (BALL_p->Y - NOW_Position.Y)*radio;
					NOW_Position.Z = NOW_Position.Z + (BALL_p->Z - NOW_Position.Z)*radio*0.3;
				}
				break;
			}
			case ROBOT_MODE_IN_RETURN:{//返回
				double dis = distance(AIM_Position,NOW_Position);
				double radio = (speed*50)/dis;
				NOW_Position.X = NOW_Position.X + (AIM_Position.X - NOW_Position.X)*radio;
				NOW_Position.Y = NOW_Position.Y + (AIM_Position.Y - NOW_Position.Y)*radio;
				NOW_Position.Z = NOW_Position.Z + (AIM_Position.Z - NOW_Position.Z)*radio*0.3;
			}
			case ROBOT_MODE_IN_MOVETO:{//移动到一个点
				double dis = distance(AIM_Position,NOW_Position);
				double radio = (speed*50)/dis;
				NOW_Position.X = NOW_Position.X + (AIM_Position.X - NOW_Position.X)*radio;
				NOW_Position.Y = NOW_Position.Y + (AIM_Position.Y - NOW_Position.Y)*radio;
				NOW_Position.Z = NOW_Position.Z + (AIM_Position.Z - NOW_Position.Z)*radio*0.3;
				break;
			}
			case ROBOT_MODE_IN_EMPTY:{//空任务
				break;
			}
		}
		//发送自己坐标系
		Simulate::SendPack(situation,Suav->uav.ID,NOW_Position);
		std::this_thread::sleep_for(std::chrono::milliseconds(15));
	}
	flag =false;
}
void runwithpath(UAVSimulate *Suav){
	double speed = 0.002;//飞行速度
	double dis;
	uint8_t situation;
	int64_t m = rand()%(Suav->path.size());
	Marker Smith;
	Suav->uav.situation = ROBOT_MODE_IN_MOVETO;
	Suav->uav.ID = AIM;
	LinkList * head=nullptr;
	LinkList * N=nullptr;
	LinkList * start=nullptr;
	for(auto &p:Suav->path){
		if(!head){
			head = new LinkList(p);
			N = head;
		}else {
			LinkList * M = new LinkList(p);
			N->next = M;
			N = M;
		}
		if(m==0){
			Suav->uav.Posion = p;
			start = N;
		}
		m--;
	}
	N->next = head;//环形链表
	while(flag){
		if((Suav->uav.situation) == ROBOT_MODE_IN_MOVETO)
		for(int i=0;i<UAV_size;i++){
			UAVSimulate* uav = Simulate::getUAV(Marker(i));
			dis = distance(uav->uav.Posion,Suav->uav.Posion);
			// dis = std::sqrt(dis*dis - (uav->uav.Posion.Z - Suav->uav.Posion.Z)*(uav->uav.Posion.Z - Suav->uav.Posion.Z));
			if(dis<speed*100){
				Smith = Marker(uav->uav.ID);
				Suav->uav.situation = ROBOT_MODE_IN_CATCH;//speed*50 最小单位
				Suav->uav.ID = uav->uav.ID;
			}
		}
		situation = Suav->uav.situation;
		switch(situation){
			case ROBOT_MODE_IN_MOVETO:{//移动
				dis = distance(start->data,Suav->uav.Posion);
				if(dis<speed*50){
					start = start->next;
					dis = distance(start->data,Suav->uav.Posion);
				}
				double radio = (speed*50)/dis;
				Suav->uav.Posion.X = Suav->uav.Posion.X + (start->data.X - Suav->uav.Posion.X)*radio;
				Suav->uav.Posion.Y = Suav->uav.Posion.Y + (start->data.Y - Suav->uav.Posion.Y)*radio;
				Suav->uav.Posion.Z = Suav->uav.Posion.Z + (start->data.Z - Suav->uav.Posion.Z)*radio;
				break;
			}
			case ROBOT_MODE_IN_CATCH://被抓到了	
				UAVSimulate* uav = Simulate::getUAV(Smith);
				Suav->uav.Posion= uav->uav.Posion;
				Suav->uav.situation = ROBOT_MODE_IN_CATCH;
				Suav->uav.ID = Smith;
				break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	while(head->next){
		if(head->next!=head){
			N = head->next;
			head->next = N->next;
			delete N;
		}else{
			delete head;
			break;
		}
	}
	flag =false;
} 
void Simulate::init(ThreadPool &pool,std::vector<std::string> port){
	if(port.size()>UAV_size){
		std::cerr<<"too much port!"<<std::endl;
		return;
	}
	for(int i=0;i<static_cast<int>(port.size());i++){
		if(!Radio[i].isOpen()){
			Radio[i].init(port[i]);
		}else{
			std::cerr<<"error open uart "<<port[i]<<std::endl;
            flag = false;
		}
		pool.enqueue(Workrun,&Radio[i], Worker);
	}	
}
void Simulate::close(){
	for(int i=0;i<Device_size;i++){
		if(Worker[i]!=nullptr){
			delete  Worker[i];
			Worker[i]=nullptr;
		}
	}
}
UAVSimulate* Simulate::addUAV(Marker ID){
	assert(ID<Device_size);
	if(Worker[ID]!=nullptr){
		return Worker[ID];
	}else{
		UAVSimulate *d = new UAVSimulate;
		Worker[ID] = d;
		return Worker[ID];
	}
	return nullptr;
}
UAVSimulate* Simulate::getUAV(Marker ID){
	if(Worker[ID]!=nullptr){
		return Worker[ID];
	}
	return nullptr;
}
void Simulate::SendPack(uint8_t CMD,uint8_t ID,Value3 data){
	std::lock_guard<std::mutex> lock(wmut);
	RadioPacket sendPacket;
	sendPacket.creatPacket(CMD);
	sendPacket.setUncharInBuffer(ID, 2);
    sendPacket.setFloatInBuffer(data.X, 3);
    sendPacket.setFloatInBuffer(data.Y, 7);
    sendPacket.setFloatInBuffer(data.Z, 11);
    Radio[ID].dataSend(sendPacket);
}

void UAVSimulate::init(ThreadPool &pool,RadioInterface *Radio,Value3 start){
	uav.situation = ROBOT_MODE_IN_INIT;
	uav.ID = 0xff;
	uav.Posion = start;
	pool.enqueue(run,Radio, this);//由于已经初始化
	return ;
}

void UAVSimulate::init(ThreadPool &pool,RadioInterface *Radio,std::vector<Value3> path){
	this->path = path;
	assert(path.size()>0);
	pool.enqueue(runwithpath,this);
}
void UAVSimulate::SetPosion(Value3 data){//设置目标位置
	uav.Posion = data;
}
bool UAVSimulate::SetSituation(uint8_t situation){//屏蔽其他状态
	if((situation&ROBOT_MODE_IN_INIT) == ROBOT_MODE_IN_INIT ||
		(situation&ROBOT_MODE_IN_TAKEOFF) == ROBOT_MODE_IN_TAKEOFF ||
		(situation&ROBOT_MODE_IN_MOVETO) == ROBOT_MODE_IN_MOVETO ||
		(situation&ROBOT_MODE_IN_LINE) == ROBOT_MODE_IN_LINE ||
		(situation&ROBOT_MODE_IN_ARCH) == ROBOT_MODE_IN_ARCH ||
		(situation&ROBOT_MODE_IN_RETURN) == ROBOT_MODE_IN_RETURN ||
		(situation&ROBOT_MODE_IN_EMPTY) == ROBOT_MODE_IN_EMPTY ){
			uav.situation = situation; 
			return true;
	}
	return false;
}

