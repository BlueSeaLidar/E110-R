/**
 * CN:
 * 主程序入口
 * 1.将SDK文件夹拷贝到自己的工程源码的当前目录
 * 2.引用standard_interface.h头文件
 *
 * EN:
 * main program entry
 * 1. Copy the files of the specified platform and the common basic files (data.h, parse.cpp.user.cpp) under the SDK to the current directory of your own project source code
 * 2. Reference data.h,udp_linux.h header file
 */

#include <stdio.h>
#include <string.h>
#include "../sdk/standard_interface.h"

// 传入回调指针的方式打印
void CallBackMsg(int msgtype, void *param, int length)
{

	switch (msgtype)
	{
	// 实时雷达点云数据
	case 1:
	{
		UserData *pointdata = (UserData *)param;

		printf("frame idx:%d  %s\t%d \t num:%d \n", pointdata->idx, pointdata->connectArg1, pointdata->connectArg2, pointdata->framedata.N);
		 /*for (int i = 0; i <1; i++)
		 {
		 	printf("%s\t%d \t%f\t%d\t%d\n", pointdata->connectArg1, pointdata->connectArg2,  pointdata->framedata.points[i].angle, pointdata->framedata.points[i].distance, pointdata->framedata.points[i].confidence);
		 }*/

		break;
	}
	// 打印信息(也可以作为日志写入)
	case 8:
	{
		char result[512];
		memcpy(result, param, length);
		printf("info: %s\n", result);
		break;
	}
	case 9:
	{
		char result[512];
		memcpy(result, param, length);
		printf("error: %s\n", result);
		break;
	}
	}
	fflush(stdout);
}
int main(int argc, char **argv)
{
	if (argc < 2)
	{
		printf("Incorrect number of parameters  %d\n usage : ./demo  ../../config/xxx.txt   At least one txt of Lidar\n", argc);
		return ARG_ERROR_NUM;
	}
	BlueSeaLidarSDK *lidarSDK = BlueSeaLidarSDK::getInstance();
	int lidar_sum = argc - 1;
	for (int i = 0; i < lidar_sum; i++)
	{
		const char *cfg_file_name = argv[i + 1];
		// 根据配置文件路径添加相关的雷达
		int lidarID = lidarSDK->addLidarByPath(cfg_file_name);
		if (!lidarID)
		{
			printf("config file is not exist:%s\n", cfg_file_name);
			return -1;
		}

		// 传入数据信息的回调函数
		lidarSDK->setCallBackPtr(lidarID, CallBackMsg);

		// 连接指定雷达，以及相关的线程池
		if (!lidarSDK->connect(lidarID))
		{
			printf("open lidar failed:%d\n", lidarID);
			return -2;
		}
		sleep(1);
		std::string model = lidarSDK->getModel(lidarID);
		std::string uuid = lidarSDK->getUUID(lidarID);
		std::string hardwareVersion = lidarSDK->hardwareVersion(lidarID);
		std::string softwareVersion = lidarSDK->softwareVersion(lidarID);
		printf("model:%s uuid:%s fversion:%s sversion:%s\n",model.c_str(),uuid.c_str(),hardwareVersion.c_str(),softwareVersion.c_str());

		//控制雷达的启动，停止，重新旋转
		 lidarSDK->setWork(lidarID,false);
		 sleep(2);
		 lidarSDK->setWork(lidarID,true);
		
	}
	while (1)
	{
		msleep(100);
	}
	return 0;
}
