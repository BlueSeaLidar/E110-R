#ifndef __LIDARDATAPROCESS_H__
#define  __LIDARDATAPROCESS_H__

#include"Global.h"

//#ifdef _WIN32

#ifdef __unix__ 
#include <sys/time.h>
#endif

 int setup_lidar_uart(int fd_uart, RunScript* arg);
 int setup_lidar_vpc(int hCom, RunScript* arg);

 bool readConfig(const char* cfg_file_name, RunScript& cfg);
 void* lidar_thread_proc_udp(void* param);
 void* lidar_thread_proc_uart(void* param);

std::vector<LidarPoint_LD> NearFilter(const std::vector<LidarPoint_LD> &tmp)  ;
#endif