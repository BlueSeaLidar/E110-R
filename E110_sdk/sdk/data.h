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
#define MAX_POINTS 16
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
	uint16_t timestamp;

};

struct RawDataHdr_LD
{
uint8_t header;
uint8_t N;
uint16_t speed;
uint16_t start_angle;
uint16_t end_angle;
LidarPoint_LD points[MAX_POINTS];
uint16_t timestamp;
};


struct FrameData_LD
{
	uint16_t N;
	LidarPoint_LD points[MAX_FRAMEPOINTS];
};

struct UserData
{
	int idx;//0表示扇区序号  1表示帧序号  超过10000000(1千万)帧回拨
	char connectArg1[16];     //ip/com
	int connectArg2;		//port /baud
	FrameData_LD framedata;	
};








#endif
