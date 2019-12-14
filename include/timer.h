#ifndef _TIMER_H
#define _TIMER_H

#include <time.h>
#include <signal.h>
#include <thread_pool.h>
#include <functional>
#include "common.h"
//设置定时器回调(通讯断链)
//设置时间统一任务

//系统定时器 检查是否断开
class timer
{
private:
	struct itimerval value,ovalue;
	time_t tt;
public:

	void SetalarmCallback(int sec,int usec,void (*fun)(int)){
		signal(SIGALRM, fun);
		value.it_value.tv_sec = sec;
		value.it_value.tv_usec = usec;
		value.it_interval.tv_sec = sec;
		value.it_interval.tv_usec = usec;
		setitimer(ITIMER_REAL,&value,&ovalue);
	}
	long long GetTime(){
		time(&tt);
		return tt;
	}
	long long DiffTime(long long t){
		time(&tt);
		return tt - t;
	}
};


#endif
