#ifndef _LIDAR_DATA
#define  _LIDAR_DATA
/**

 * Copyright (C),  Pacecat:(C) <LanHai>,All right reserved

 * File name:      data.h

 * Author:  	   *
      
 * Version:        1.0  
    
 * Date:		   2022.3.25

 * Description:    Data storage structure and data parsing capabilities for all devices

 */

#include <stdint.h>
#include <string.h>
#include <string>
#include<stdio.h>
#include <stdlib.h>
#include<vector>

#define PI 3.1415926535898
#define MAX_LIDARS 8
#define MAX_FANS 120
#define MAX_POINTS 12
#define MAX_FRAMEPOINTS 1024
#define BUF_SIZE 8*1024
#define SLEEP_SIZE 5
#define USER_SIZE  10

#define getbit(x,y)   ((x) >> (y)&1)
#define setbit(x,y) x|=(1<<y)         //将X的第Y位置1
#define clrbit(x,y) x&=~(1<<y)            //将X的第Y位清0
#define UNUSED(x) (void)(x)

#define ANGLE_PER_FRAME 12
#define HEADER 0x54

struct LidarPoint_LD
{
    uint16_t distance;
	float angle;
    uint8_t confidence;
};

struct RawDataHdr_LD
{
uint8_t header;
uint8_t N;
uint16_t speed;
uint16_t start_angle;
uint16_t end_angle;
LidarPoint_LD points[MAX_POINTS];
//uint16_t timestamp;
};

struct FrameData_LD
{
	uint16_t N;
	LidarPoint_LD points[MAX_FRAMEPOINTS];
};

struct Range
{
	double min;
	double max;
};
struct ConnectArg
{
	std::string scan_topics;
	std::string cloud_topics;
	std::string arg1;
	int arg2;
};

struct ArgData
{
	std::string frame_id;
	int dev_id;
	ConnectArg connectargs;
	bool output_scan;
	bool output_cloud;
	bool output_cloud2;
	bool inverted;
	bool reversed;
	bool with_angle_filter;
	double min_angle;
	double max_angle;
	double min_dist;
	double max_dist;
	std::vector<Range> masks;
	std::string type;
	int num;
};


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
	ACTION action;//0 无操作   1.启停控制指令  2.获取设备全局参数  3.设置设备参数  4 读取防区   5 设置防区
	//当前传入的指令
	int mode;
	int send_len;
	char send_cmd[1024];
	int recv_len;
	char recv_cmd[1024];
	int Nfans;
	pthread_mutex_t mtx;
	RawDataHdr_LD fandatas[MAX_FANS];
	RawDataHdr_LD tmpfandata;
	int frameidx;
	FrameData_LD framedata;	
	FrameData_LD nextframedata;	//出现分割情况的后半包
	RunScript  runscript;
	
};



#endif
