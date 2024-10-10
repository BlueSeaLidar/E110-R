#include "LidarDataProcess.h"
#include "error.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#ifdef __unix__
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <stdarg.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#endif

void *lidar_thread_proc_uart(void *param)
{
	RunConfig *cfg = (RunConfig *)param;
	strcpy(cfg->userdata.connectArg1, cfg->runscript.connectArg);
	cfg->userdata.connectArg2 = cfg->runscript.connectArg2;
	RawDataHdr_LD dat;
	int is_pack;
	std::string error;
	char result[512] = {0};
	char info[512] = {0};
	int consume = 0;
	FrameData_LD framedata;
	FrameData_LD tmpdata;
	memset(&framedata, 0, sizeof(FrameData_LD));
	memset(&tmpdata, 0, sizeof(FrameData_LD));
	RawDataHdr_LD tmpraw;
	cfg->userdata.idx = 0;
	setup_lidar_uart(cfg->fd, &cfg->runscript);

	/*
	 * 4, read and parser data
	 */
	unsigned char *buf = new unsigned char[BUF_SIZE];
	int buf_len = 0;
	int ret=0;
	struct timeval start_tv;
	gettimeofday(&start_tv, NULL);

	while (cfg->state != UNINIT)
	{
#ifdef _WIN32
		if (cfg->fd > 0)
#elif __linux
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(cfg->fd, &fds);
		struct timeval to = {1, 1};
		ret = select(cfg->fd + 1, &fds, NULL, NULL, &to);
		if (ret < 0)
		{
			printf("select error\n");
			return NULL;
		}
		if (cfg->fd > 0 && FD_ISSET(cfg->fd, &fds))
#endif // _WIN32
		{
			int nr = read(cfg->fd, buf + buf_len, BUF_SIZE - buf_len);
			if (nr < 0)
			{
				sprintf(info, "read port %d error %d", buf_len, nr);
				cfg->callback(9, info, strlen(info) + 1);
				break;
			}
			if (nr == 0)
			{
#ifdef _WIN32
				msleep(10);
#endif
				//continue;
			}
			if (nr > 0)
			{
				buf_len += nr;
				//printf("%d %d %d\n",nr,buf_len,consume);
			}
		}
		while (buf_len >=47)
		{
			memset(result, 0, sizeof(result));
			memset(&dat, 0, sizeof(RawDataHdr_LD));
			int nfan = 1;
			is_pack = LD::parse_data(buf_len, buf, &dat, consume, result);
			if (is_pack == 1 && dat.N > 0)
			{
				//int ret = LD::whole_data_process(dat, cfg->userdata.framedata, tmpdata, error);
				int ret = LD::whole_data_process(&dat, nfan, tmpraw, cfg->userdata.framedata);
				if (ret == 1)
				{
					if (cfg->action < ONLINE)
						cfg->action = ONLINE;

					cfg->userdata.idx++;
					cfg->callback(1, &cfg->userdata, sizeof(UserData));

					// 跨帧分割的包赋值
					if (tmpdata.N > 0)
					{
						memcpy(&cfg->userdata.framedata, &tmpdata, sizeof(FrameData_LD));
						// printf("%s %d %d\n",__FUNCTION__,__LINE__,cfg->userdata.framedata.N);
					}
					cfg->userdata.framedata.N = 0;
				}
				else if (ret == -1)
				{
					cfg->callback(9, (char*)error.c_str(), error.length());
				}
				memset(&tmpdata, 0, sizeof(FrameData_LD));
			}
			else if (is_pack == -1)
			{
				cfg->userdata.framedata.N = 0;
			}
			if (consume > 0)
			{
				 //data is not whole fan,drop it
				 if (!is_pack)
				 {
				 	printf("drop %d bytes: %02x %02x %02x %02x %02x %02x \n",
				 		   consume,
				 		   buf[0], buf[1], buf[2],
				 		   buf[3], buf[4], buf[5]);
				 }

				for (int i = consume; i < buf_len; i++)
					buf[i - consume] = buf[i];
				buf_len -= consume;
			}
			//
			/*if (cfg->action > ONLINE)
			{
				printf("%s %d %d\n", __FUNCTION__, __LINE__, cfg->action);
				break;
			}*/
				
		}
		// 存在需要操作的指令
		switch (cfg->action)
		{
		case CONTROL:
		{
			SystemAPI::Write(cfg->fd, cfg->send_cmd, cfg->send_len);
			printf("%s %d %s\n", __FUNCTION__, __LINE__, cfg->send_cmd);
			cfg->action = FINISH;
			break;
		}
		case QUERY:
		{
			//SystemAPI::Read(cfg->fd, buf, sizeof(buf));
			std::string cmd = std::string(cfg->send_cmd, cfg->send_len);
			char response[128] = {};
			
			if (cmd == "LUUIDH")
			{
				for (int i = 3; i >= 0; i--)
				{
					if (CommunicationAPI::uart_talk(cfg->fd, cmd.size(), cmd.c_str(), 11, "PRODUCT SN:", 20, response,3))
					{
						std::string sn = BaseAPI::stringfilter(response, 20);
						strcpy(cfg->recv_cmd,sn.c_str());
						cfg->recv_len=20;
						printf("%s %d %s\n",__FUNCTION__,__LINE__,cfg->recv_cmd);
						cfg->action = FINISH;
						break;
					}
					else
					{
						printf("%s %d %s  err\n",__FUNCTION__,__LINE__,cfg->recv_cmd);
					}
				}
			}
			else if (cmd == "LVERSH")
			{
				for (int i = 3; i >= 0; i--)
				{
					if(CommunicationAPI::uart_talk(cfg->fd, cmd.size(), cmd.c_str(), 12, "MCU VERSION:", 32, response,3))
                	{
                    std::string version = BaseAPI::stringfilter(response,32);
					strcpy(cfg->recv_cmd,version.c_str());
					cfg->recv_len=32;
					printf("%s %d %s %ld\n",__FUNCTION__,__LINE__,cfg->recv_cmd,version.size());
					cfg->action = FINISH;
                    break;
                	}
					else
					{
						printf("%s %d %s  err\n",__FUNCTION__,__LINE__,cfg->recv_cmd);
					}
				}
				
			}
			else if (cmd == "LTYPEH")
			{
				for (int i = 3; i >= 0; i--)
				{
					if (CommunicationAPI::uart_talk(cfg->fd, cmd.size(), cmd.c_str(), 8, "TYPE ID:", 16, response,3))
					{
						std::string sn = BaseAPI::stringfilter(response, 16);
						//printf("%s %d %s\n",__FUNCTION__,__LINE__,sn.c_str());
						strcpy(cfg->recv_cmd,sn.c_str());
						cfg->recv_len=16;
						printf("%s %d %s %d\n",__FUNCTION__,__LINE__,cfg->recv_cmd,sn.size());
						cfg->action = FINISH;
						break;
					}
					else
					{
						printf("%s %d %s  err\n",__FUNCTION__,__LINE__,cfg->recv_cmd);
					}
				}
				
			}
			cfg->action = FINISH;
			//memset(cfg->recv_cmd,0,sizeof(1024));
			break;
		}
		case ONLINE:
		{
			// 说明运行正常
			break;
		}
		default:
			break;
		}
	}
	SystemAPI::closefd(cfg->fd, false);
	return 0;
}

int setup_lidar_uart(int fd_uart, RunScript *arg)
{
	char buf[32];
	int index = 3;
	int nr = 0;

	write(fd_uart, "LSTARH", 6);
	for (int i = 0; i < 300 && nr <= 0; i++)
	{
		msleep(10);
		nr = read(fd_uart, buf, sizeof(buf));
	}
	if (nr <= 0)
	{
		printf("serial port seem not working\n");
		SystemAPI::closefd(fd_uart, false);
		return READ_UART_FAILED;
	}
	if (arg->uuid >= 0)
	{
		for (int i = 0; i < index; i++)
		{
			if (CommunicationAPI::uart_talk(fd_uart, 6, "LUUIDH", 11, "PRODUCT SN:", 20, buf))
			{
				printf("LiDAR uuid info:  %s\n", BaseAPI::stringfilter(buf, 20).c_str());
				break;
			}
		}
	}
	// 硬件版本号
	if (arg->version >= 0)
	{
		for (int i = 0; i < index; i++)
		{
			if (CommunicationAPI::uart_talk(fd_uart, 6, "LVERSH", 14, "MOTOR VERSION:", 12, buf))
			{
				printf("LiDAR version info:  %.12s\n", buf);
				break;
			}
		}
	}
	if (arg->model >= 0)
	{
		for (int i = 0; i < index; i++)
		{
			if (CommunicationAPI::uart_talk(fd_uart, 6, "LTYPEH", 8, "TYPE ID:", 16, buf))
			{
				std::string tmp = BaseAPI::stringfilter(buf, 16);
				printf("LiDAR model type  %s \n", tmp.c_str());
				break;
			}
		}
	}
	return 0;
}

#include <cctype>
#include <algorithm>
bool readConfig(const char *cfg_file_name, RunScript &cfg)
{
	std::ifstream infile;
	infile.open(cfg_file_name);
	if (!infile.is_open())
		return false;

	std::string str, key, value;
	while (getline(infile, str))
	{
		std::stringstream linestream(str);
		getline(linestream, key, ':');
		if (key.find("#") != std::string::npos || key.find(" ") != std::string::npos)
			continue;
		getline(linestream, value);
		if (!key.empty())
			value.erase(std::remove_if(value.begin(), value.end(), ::isspace), value.end());

		if (key == "type")
			strcpy(cfg.type, value.c_str());
		else if (key == "connectArg")
			strcpy(cfg.connectArg, value.c_str());
		else if (key == "connectArg2")
			cfg.connectArg2 = atoi(value.c_str());
		else if (key == "uuid")
			cfg.uuid = atoi(value.c_str());
		else if (key == "model")
			cfg.model = atoi(value.c_str());
		else if (key == "version")
			cfg.version = atoi(value.c_str());
	}
	return true;
}
