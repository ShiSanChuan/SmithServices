#include "UAVSimulate.h"
#include "PathGeneration.h"
#include <random>
//模拟一个UAV飞行哦
//
//
RadioInterface Simulate::Radio;
static std::mutex rmut;
static std::mutex wmut;
UAVSimulate* Simulate::Worker[Device_size];
//UAV只会收到移动的命令
//移动命令优先级最高 
static void Workrun(RadioInterface *Radio,UAVSimulate** Worker){
    Value3 data;
    RadioPacket recvPacket;
	while(flag){
		if(Radio->dataRecv(recvPacket)==0){
			unsigned char CMD = recvPacket.getCMD();
			if((CMD&0xf0)>0x80){

			}else{
				std::lock_guard<std::mutex> lock(rmut);
				unsigned char device = CMD&0x0f;
				if(device!=AIM&&Worker[device]){
					data.X = recvPacket.getFloatInBuffer(2);
					data.Y = recvPacket.getFloatInBuffer(6);
					data.Z = recvPacket.getFloatInBuffer(10);
					if(Worker[device]->SetSituation(CMD))
						Worker[device]->SetPosion(data);//设定目标
					
				}
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
	flag = false;
}
void run(RadioInterface *Radio,UAVSimulate *Suav){
	float speed = UAV_speed;//飞行速度 km/h -> 1/360 m/ms 
	float field = UAV_filed;	 // 可以看见球的范围
	Value3 backaim;
	//返回点
	backaim.X = 0;
	backaim.Y = 0;
	backaim.Z = 0;
	bool return_flag = false;
	while(flag){
		// std::lock_guard<std::mutex> lock(rmut);
		if(return_flag)Suav->uav.situation = (Suav->uav.situation&0x0f)|ROBOT_MODE_IN_RETURN;
		if((Suav->uav.situation&0xf0)!=0xf0&&
			(Suav->uav.situation&ROBOT_MODE_IN_RETURN)!=ROBOT_MODE_IN_RETURN){
			Suav->uav.situation = (Suav->uav.situation&0x0f)|ROBOT_MODE_IN_MOVE;
		}
		if((Suav->uav.situation&0xf0)!=0xf0&&
		(Suav->uav.situation&ROBOT_MODE_IN_RETURN)!=ROBOT_MODE_IN_RETURN){//已初始化启动飞机 检测下方摄像头是否有待扎破气球 优先级较高
			for(int i=0;i<BALLON_num;i++){
				if(BALLON_Posion[i].Z == 0)continue;//已被击落气球
				float dis = distance(Suav->uav.Posion,BALLON_Posion[i]);
				if( dis<speed*50){//已扎到
					BALLON_Posion[i].Z = 0;
					Suav->uav.situation = (Suav->uav.situation&0x0f)|ROBOT_MODE_IN_MOVE;
					break;
				}
				if(dis<UAV_low_filed){
					Suav->uav.situation = (Suav->uav.situation&0x0f)|ROBOT_MODE_IN_STAB;//speed*50 最小单位
					float radio = (speed*50)/dis;
					Suav->uav.Posion.X = Suav->uav.Posion.X + (BALLON_Posion[i].X - Suav->uav.Posion.X)*radio;
					Suav->uav.Posion.Y = Suav->uav.Posion.Y + (BALLON_Posion[i].Y - Suav->uav.Posion.Y)*radio;
					Suav->uav.Posion.Z = Suav->uav.Posion.Z + (BALLON_Posion[i].Z - Suav->uav.Posion.Z)*radio;
					break;
				}
			}
		}
		if((Suav->uav.situation&0xf0)!=0xf0&&
		(Suav->uav.situation&ROBOT_MODE_IN_RETURN)!=ROBOT_MODE_IN_RETURN){//已初始化启动飞机 检查上方摄像头是否有球，优先级高
			UAVSimulate* aim = Simulate::getUAV(AIM);
			if((aim->uav.situation&ROBOT_MODE_IN_CATCH)==ROBOT_MODE_IN_CATCH)break;//已被抓到
			bool other_incatch = false;
			for(int i=0x01;i<0x08;i=i<<1){
				if((Suav->uav.situation&0x0f)!=i){
					auto other = Simulate::getUAV(Marker(i));
					if((other->uav.situation&ROBOT_MODE_IN_CATCH)==ROBOT_MODE_IN_CATCH||
						(other->uav.situation&ROBOT_MODE_IN_RETURN)==ROBOT_MODE_IN_RETURN){
						other_incatch = true;
						break;
					}
				}
			}
			if(!other_incatch){
				// if((aim->uav.situation&0xf0) ==ROBOT_MODE_IN_CATCH &&
				//  (aim->uav.situation&0x0f)!=(Suav->uav.situation&0x0f) )break;//已经被抓到了
				float dis = distance(aim->uav.Posion,Suav->uav.Posion);
				// dis = std::sqrt(dis*dis - (aim->uav.Posion.Z - Suav->uav.Posion.Z)*(aim->uav.Posion.Z - Suav->uav.Posion.Z));
				if(std::abs(aim->uav.Posion.X - Suav->uav.Posion.X)<0.5 &&std::abs(aim->uav.Posion.Y - Suav->uav.Posion.Y)<0.5  ){//快要抓到
					Suav->uav.situation = (Suav->uav.situation&0x0f)|ROBOT_MODE_IN_RETURN;//speed*50 最小单位
					Suav->UAV_AIM = backaim;
					Simulate::SendPack(ROBOT_MODE_IN_RETURN|AIM, aim->uav.Posion);
					Suav->uav.situation = (Suav->uav.situation&0x0f)|ROBOT_MODE_IN_RETURN;
					return_flag = true;
				}
				else if(dis<std::abs(aim->uav.Posion.Z - Suav->uav.Posion.Z)*1.2){//视野里 field
					UAVSimulate* aim = Simulate::getUAV(AIM);
					float radio = (speed*50)/dis;
					Suav->uav.Posion.X = Suav->uav.Posion.X + (aim->uav.Posion.X - Suav->uav.Posion.X)*radio;
					Suav->uav.Posion.Y = Suav->uav.Posion.Y + (aim->uav.Posion.Y - Suav->uav.Posion.Y)*radio;
					Suav->uav.Posion.Z = Suav->uav.Posion.Z + (aim->uav.Posion.Z - Suav->uav.Posion.Z)*radio*0.3;
					// printf("%f %f %f\n",aim->uav.Posion.X,aim->uav.Posion.Y,aim->uav.Posion.Z);
					//发送目标坐标系
					Simulate::SendPack(ROBOT_MODE_IN_CATCH|AIM, aim->uav.Posion);
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
					Suav->uav.situation = (Suav->uav.situation&0x0f)|ROBOT_MODE_IN_CATCH;
				}
			}
		}
		uint8_t situation = Suav->uav.situation&0xf0;
		if(return_flag) situation = ROBOT_MODE_IN_RETURN;
		switch(situation){
			case ROBOT_MODE_IN_INIT://初始化启动
				std::this_thread::sleep_for(std::chrono::seconds(3));//飞机启动
				Suav->uav.Posion.X = Suav->UAV_AIM.X;
				Suav->uav.Posion.Y = Suav->UAV_AIM.Y;
				Suav->uav.Posion.Z = Suav->UAV_AIM.Z;
				Suav->uav.situation = (Suav->uav.situation&0x0f)|ROBOT_MODE_IN_MOVE;
				printf("UAV%d start! \n",Suav->uav.situation&0x0f);
				break;
			case ROBOT_MODE_IN_RETURN:{//返回
				float dis = distance(Suav->UAV_AIM,Suav->uav.Posion);
				float radio = (speed*50)/dis;
				Suav->uav.Posion.X = Suav->uav.Posion.X + (backaim.X - Suav->uav.Posion.X)*radio;
				Suav->uav.Posion.Y = Suav->uav.Posion.Y + (backaim.Y - Suav->uav.Posion.Y)*radio;
				Suav->uav.Posion.Z = Suav->uav.Posion.Z + (backaim.Z - Suav->uav.Posion.Z)*radio*0.2;
				Suav->uav.situation = (Suav->uav.situation&0x0f)|ROBOT_MODE_IN_RETURN;
				break;
			}
			case ROBOT_MODE_IN_CATCH://抓气球
				break;
			case ROBOT_MODE_IN_STAB://刺气球	
				break;
			case ROBOT_MODE_IN_MOVE:{//移动
				float dis = distance(Suav->UAV_AIM,Suav->uav.Posion)+0.000001;
				float radio = (speed*50)/dis;
				Suav->uav.Posion.X = Suav->uav.Posion.X + (Suav->UAV_AIM.X - Suav->uav.Posion.X)*radio;
				Suav->uav.Posion.Y = Suav->uav.Posion.Y + (Suav->UAV_AIM.Y - Suav->uav.Posion.Y)*radio;
				Suav->uav.Posion.Z = Suav->uav.Posion.Z + (Suav->UAV_AIM.Z - Suav->uav.Posion.Z)*radio;
				break;
			}
		}
		//发送自己坐标系
		if((Suav->uav.situation&0xf0) == ROBOT_MODE_IN_CATCH||
			(Suav->uav.situation&0xf0) == ROBOT_MODE_IN_STAB||
			(Suav->uav.situation&0xf0) == ROBOT_MODE_IN_MOVE||
			(Suav->uav.situation&0xf0) == ROBOT_MODE_IN_RETURN){
			Simulate::SendPack(Suav->uav.situation,Suav->uav.Posion);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(15));
	}
	flag =false;
}
void runwithpath(UAVSimulate *Suav){
	float speed = 0.00012;//飞行速度
	float dis;
	uint8_t situation;
	int64_t m = rand()%(Suav->path.size());
	Marker Smith;
	Suav->uav.situation = ROBOT_MODE_IN_MOVE;
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
		if((Suav->uav.situation&0xf0) == ROBOT_MODE_IN_MOVE)
		for(int i=0x01;i<0x08;i=i<<1){
			UAVSimulate* uav = Simulate::getUAV(Marker(i));
			dis = distance(uav->uav.Posion,Suav->uav.Posion);
			// dis = std::sqrt(dis*dis - (uav->uav.Posion.Z - Suav->uav.Posion.Z)*(uav->uav.Posion.Z - Suav->uav.Posion.Z));
			if(dis<speed*100){
				Smith = Marker(uav->uav.situation&0x0f);
				Suav->uav.situation = ROBOT_MODE_IN_CATCH|Smith;//speed*50 最小单位
			}
		}
		situation = Suav->uav.situation&0xf0;
		switch(situation){
			case ROBOT_MODE_IN_MOVE:{//移动
				dis = distance(start->data,Suav->uav.Posion);
				if(dis<speed*50){
					start = start->next;
					dis = distance(start->data,Suav->uav.Posion);
				}
				float radio = (speed*50)/dis;
				Suav->uav.Posion.X = Suav->uav.Posion.X + (start->data.X - Suav->uav.Posion.X)*radio;
				Suav->uav.Posion.Y = Suav->uav.Posion.Y + (start->data.Y - Suav->uav.Posion.Y)*radio;
				Suav->uav.Posion.Z = Suav->uav.Posion.Z + (start->data.Z - Suav->uav.Posion.Z)*radio;
				break;
			}
			case ROBOT_MODE_IN_CATCH://被抓到了	
				UAVSimulate* uav = Simulate::getUAV(Smith);
				Suav->uav.Posion= uav->uav.Posion;
				Suav->uav.situation = ROBOT_MODE_IN_CATCH|Smith;
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
void Simulate::init(ThreadPool &pool,std::string port){
	if(!Radio.isOpen()){
		Radio.init(port);
	}
	pool.enqueue(Workrun,&Radio, Worker);
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
void Simulate::SendPack(uint8_t CMD,Value3 data){
	std::lock_guard<std::mutex> lock(wmut);
	RadioPacket sendPacket;
	sendPacket.creatPacket(CMD);
    sendPacket.setFloatInBuffer(data.X, 2);
    sendPacket.setFloatInBuffer(data.Y, 6);
    sendPacket.setFloatInBuffer(data.Z, 10);
    Radio.dataSend(sendPacket);
}

void UAVSimulate::init(ThreadPool &pool,RadioInterface *Radio,Value3 start){
	uav.situation = 0xff;
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
	UAV_AIM = data;
}
bool UAVSimulate::SetSituation(uint8_t situation){//屏蔽其他状态
	if((situation&ROBOT_MODE_IN_INIT) == ROBOT_MODE_IN_INIT ||
		(situation&ROBOT_MODE_IN_MOVE) == ROBOT_MODE_IN_MOVE){
			uav.situation = situation; 
			return true;
	}
	return false;
}

