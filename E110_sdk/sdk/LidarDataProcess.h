#ifndef __LIDARDATAPROCESS_H__
#define  __LIDARDATAPROCESS_H__

#include"Global.h"

//#ifdef _WIN32

#ifdef __unix__ 
#include <sys/time.h>
#endif
typedef void (*printfMsg)(int, void*,int);

enum ACTION
{
	OFFLINE=0,
	ONLINE,
	RUN,
	CONTROL,
	QUERY,
	FINISH
};
enum STATE
{
	INIT=0,
	WORK,
	UNINIT
};

struct Responsive
{
	int mode;//0x0043  0x0053  0x4753
	int send_len;
	char send_cmd[1024];
	short rand;
	long timestamp;
};

struct RunScript
{
	//connect
	char type[16];//"uart"   or   "udp"  "vpc"
	char connectArg[16];//ip/com
	int connectArg2;//port/baud
	//get
	int uuid;
	int model;
	int version;
};

//运行配置 
struct RunConfig
{	
	int ID;
	STATE state;	//-1 stop all 0 init   1 run work thread  2 run work and web thread 

	unsigned long thread_data;//DWORD == pthread_t
	int fd;//句柄
	printfMsg  callback;	//CN:信息打印回调函数					EN:Information printing callback function
	UserData userdata;//当前帧
	ACTION action;//0 无操作   1.启停控制指令  2.获取设备全局参数  3.设置设备参数  4 读取防区   5 设置防区
	//当前传入的指令
	int mode;
	int send_len;
	char send_cmd[1024];
	int recv_len;
	char recv_cmd[1024];

	RunScript  runscript;
	
};

 int setup_lidar_uart(int fd_uart, RunScript* arg);
 int setup_lidar_vpc(int hCom, RunScript* arg);

 bool readConfig(const char* cfg_file_name, RunScript& cfg);
 bool checkPointsLengthZero(UserData *tmp, float scale);
 void* lidar_thread_proc_udp(void* param);
 void* lidar_thread_proc_uart(void* param);


#endif