#pragma once
#include"data.h"
#include <set>
#include<math.h>
#ifdef _WIN32
#pragma warning(disable:4996)
#include<Windows.h>
#ifndef sleep(sec)   Sleep(sec * 1000)
#define sleep(sec)   Sleep(sec * 1000)
#endif
#define msleep(msec) Sleep(msec)
#elif __linux
#include<sys/time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <termios.h>
#include<dirent.h>
#include<limits>
#include<float.h>
#define msleep(msec) usleep(msec * 1000)
#endif
struct UARTARG
{
    char portName[16];
    int port;
};
namespace BaseAPI {
    bool checkAndMerge(int type, char* ip, char* mask, char* gateway, int port, char* result);
    std::string stringfilter(char *str,int num);
}

namespace SystemAPI {
    int GetComList(std::vector<UARTARG>& list);
    int closefd(int __fd, bool isSocket);
    std::vector<std::string> GetComPort();
    int open_serial_port(const char* name, int speed);
    int open_socket_port(int localhost);

    int Read(int __fd, void* __buf, int __nbytes);
    int Write(int __fd, const void* __buf, int __n);


}
int GetDevInfoByVPC(const char* port_str, int speed);
int GetDevInfoByUART(const char* port_str, int speed);

namespace CommunicationAPI {
    bool uart_talk(int hCom, int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch,int waittime=100); 
}



unsigned int stm32crc(unsigned int* ptr, unsigned int len);

#ifdef _WIN32
    void gettimeofday(timeval* tv, void*);
#endif


#ifdef _WIN32
    int Open_serial_port(const char* name, int port);
    int read(int __fd, void* __buf, int __nbytes);
    int write(int __fd, const void* __buf, int __n);
#elif __linux__
    extern "C"  int change_baud(int fd, int baud);
    int Open_serial_port(const char* name, int port);
#endif


namespace LD {
    uint8_t CalCRC8(uint8_t *p, uint8_t len);
    int parse_data(int len, unsigned char* buf, RawDataHdr_LD*dat,int& consume,char*result);
    // 1  ok    0 continue  -1  error
    int whole_data_process(RawDataHdr_LD *raws, int &nfan, RawDataHdr_LD &tmpraw,FrameData_LD &framedata);
}
