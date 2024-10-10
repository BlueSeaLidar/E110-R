#include "../include/LidarDataProcess.h"
#include "../include/usererror.h"
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

double curr_speed_=0;
int scan_frequency_ = 4500;
int intensity_low_ = 15;
int intensity_single_ = 220;

void *lidar_thread_proc_uart(void *param)
{
	RunConfig *cfg = (RunConfig *)param;
	RawDataHdr_LD dat;
	int is_pack;
	std::string error;
	char result[512] = {0};
	int consume = 0;
	FrameData_LD framedata;
	FrameData_LD tmpdata;
	memset(&framedata, 0, sizeof(FrameData_LD));
	memset(&tmpdata, 0, sizeof(FrameData_LD));
	setup_lidar_uart(cfg->fd, &cfg->runscript);
	DEBUG("lidar is run ok");
	/*
	 * 4, read and parser data
	 */
	unsigned char *buf = new unsigned char[BUF_SIZE];
	int buf_len = 0;
	int ret;
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
				DEBUG("read port %d error %d", buf_len, nr);
				// break;
			}
			if (nr == 0)
			{
#ifdef _WIN32
				msleep(10);
#endif
				if (nr < BUF_SIZE - 10)
					usleep(10000 - nr * 10);
				continue;
			}
			if (nr > 0)
			{
				buf_len += nr;
				//DEBUG("%d  %d  %d\n", nr, buf_len,consume);
			}
		}

		while (buf_len >=47)
		{
			memset(result, 0, sizeof(result));
			memset(&dat, 0, sizeof(RawDataHdr_LD));
			is_pack = LD::parse_data(buf_len, buf, &dat, consume, result);
			if (consume > 0)
			{
				for (int i = consume; i < buf_len; i++)
					buf[i - consume] = buf[i];
				buf_len -= consume;
			}
			if (cfg->Nfans >= 120)
			{
				DEBUG("read too fast!\n");
				continue;
			}	
			if (is_pack == 1 && dat.N > 0)
			{
				if(cfg->action < ONLINE)
					cfg->action=ONLINE;


				//DEBUG("%f %f %f %d",dat.points[0].angle,dat.points[1].angle,dat.points[dat.N-1].angle,dat.N);
				// curr_speed_=dat.speed;
				// std::vector<LidarPoint_LD> tmp;//NearFilter;
				// for(int i=0;i<dat.N;i++)
				// 	tmp.push_back(dat.points[i]);
				// std::vector<LidarPoint_LD> result = NearFilter(tmp);
				// for(int i=0;i<result.size();i++)
				// 	memcpy(&dat.points[i], &result[i], sizeof(LidarPoint_LD));

				// DEBUG("%f %f %d",dat.points[0].angle,dat.points[result.size()-1].angle,result.size());

				pthread_mutex_lock(&cfg->mtx);
				memcpy(&cfg->fandatas[cfg->Nfans], &dat, sizeof(RawDataHdr_LD));
				cfg->Nfans++;
				pthread_mutex_unlock(&cfg->mtx);
			}
			
		}
		// 存在需要操作的指令
		switch (cfg->action)
		{
		case CONTROL:
		{
			SystemAPI::Write(cfg->fd, cfg->send_cmd, cfg->send_len);
			cfg->action = FINISH;
			break;
		}
		case QUERY:
		{
			// SystemAPI::Read(cfg->fd, buf, sizeof(buf));
			std::string cmd = std::string(cfg->send_cmd, cfg->send_len);
			char response[128] = {};

			if (cmd == "LUUIDH")
			{
				for (int i = 3; i >= 0; i--)
				{
					if (CommunicationAPI::uart_talk(cfg->fd, cmd.size(), cmd.c_str(), 11, "PRODUCT SN:", 20, response, 3))
					{
						std::string sn = BaseAPI::stringfilter(response, 20);
						strcpy(cfg->recv_cmd, sn.c_str());
						cfg->recv_len = 20;
						printf("%s %d %s\n", __FUNCTION__, __LINE__, cfg->recv_cmd);
						cfg->action = FINISH;
						break;
					}
					else
					{
						printf("%s %d %s  err\n", __FUNCTION__, __LINE__, cfg->recv_cmd);
					}
				}
			}
			else if (cmd == "LVERSH")
			{
				for (int i = 3; i >= 0; i--)
				{
					if (CommunicationAPI::uart_talk(cfg->fd, cmd.size(), cmd.c_str(), 12, "MCU VERSION:", 32, response, 3))
					{
						std::string version = BaseAPI::stringfilter(response, 32);
						strcpy(cfg->recv_cmd, version.c_str());
						cfg->recv_len = 32;
						printf("%s %d %s %ld\n", __FUNCTION__, __LINE__, cfg->recv_cmd, version.size());
						cfg->action = FINISH;
						break;
					}
					else
					{
						printf("%s %d %s  err\n", __FUNCTION__, __LINE__, cfg->recv_cmd);
					}
				}
			}
			else if (cmd == "LTYPEH")
			{
				for (int i = 3; i >= 0; i--)
				{
					if (CommunicationAPI::uart_talk(cfg->fd, cmd.size(), cmd.c_str(), 8, "TYPE ID:", 16, response, 3))
					{
						std::string sn = BaseAPI::stringfilter(response, 16);
						// printf("%s %d %s\n",__FUNCTION__,__LINE__,sn.c_str());
						strcpy(cfg->recv_cmd, sn.c_str());
						cfg->recv_len = 16;
						// printf("%s %d %s %d\n",__FUNCTION__,__LINE__,cfg->recv_cmd,sn.size());
						cfg->action = FINISH;
						break;
					}
				}
			}
			cfg->action = FINISH;
			// memset(cfg->recv_cmd,0,sizeof(1024));
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
		DEBUG("serial port seem not working\n");
		SystemAPI::closefd(fd_uart, false);
		return -1;
	}
	if (arg->uuid >= 0)
	{
		for (int i = 0; i < index; i++)
		{
			if (CommunicationAPI::uart_talk(fd_uart, 6, "LUUIDH", 11, "PRODUCT SN:", 20, buf))
			{
				DEBUG("LiDAR uuid info:  %s\n", BaseAPI::stringfilter(buf, 20).c_str());
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
				DEBUG("LiDAR version info:  %.12s\n", buf);
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
				DEBUG("LiDAR model type  %s \n", tmp.c_str());
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


std::vector<LidarPoint_LD> NearFilter(const std::vector<LidarPoint_LD> &tmp)  
{
  std::vector<LidarPoint_LD> normal, pending, item;
  std::vector<std::vector<LidarPoint_LD>> group;

  // Remove points within 5m
  for (auto n : tmp) {
    if (n.distance < 5000) {
      pending.push_back(n);
    } else {
      normal.push_back(n);
    }
  }

  if (tmp.empty()) return normal;

  double angle_delta_up_limit = curr_speed_ / scan_frequency_ * 2;
  //printf("%lf %d %d \n",angle_delta_up_limit,intensity_low_,intensity_single_);
  // sort
  std::sort(pending.begin(), pending.end(),
            [](LidarPoint_LD a, LidarPoint_LD b) { return a.angle < b.angle; });

  LidarPoint_LD last;
  memset(&last,0,sizeof(LidarPoint_LD));
  last.angle=-10;

  // group
  for (auto n : pending) {
    if (fabs(n.angle - last.angle) > angle_delta_up_limit ||
        fabs(n.distance - last.distance) > last.distance * 0.03) {
      if (item.empty() == false) {
        group.push_back(item);
        item.clear();
      }
    }
    item.push_back(n);
    last = n;
  }
  // push back last item
  if (item.empty() == false) group.push_back(item);

  if (group.empty()) return normal;

  // Connection 0 degree and 359 degree
  auto first_item = group.front().front();
  auto last_item = group.back().back();
  if (fabs(first_item.angle + 360.f - last_item.angle) < angle_delta_up_limit &&
      fabs(first_item.distance - last_item.distance) < last.distance * 0.03) {
    group.front().insert(group.front().begin(), group.back().begin(), group.back().end());
    group.erase(group.end() - 1);
  }
  // selection
  for (auto n : group) {
    if (n.size() == 0) continue;
    // No filtering if there are many points
    if (n.size() > 15) {
      normal.insert(normal.end(), n.begin(), n.end());
      continue;
    }

    // Filter out those with few points
    if (n.size() < 3) {
      int c = 0;
      for (auto m : n) {
        c += m.confidence;
      }
      c /= n.size();
      if (c < intensity_single_) {
        for (auto& point: n) {
          point.distance = 0;
          point.confidence = 0;
        }
        normal.insert(normal.end(), n.begin(), n.end());
        continue;
      }
    }

    // Calculate the mean value of distance and intensity
    double intensity_avg = 0;
    double dis_avg = 0;
    for (auto m : n) {
      intensity_avg += m.confidence;
      dis_avg += m.distance;
    }
    intensity_avg /= n.size();
    dis_avg /= n.size();

    // High intensity, no filtering
    if (intensity_avg > intensity_low_) {
      normal.insert(normal.end(), n.begin(), n.end());
      continue;
    } else {
      for (auto& point : n) {
        point.distance = 0;
        point.confidence = 0;
      }
      normal.insert(normal.end(), n.begin(), n.end());
      continue;
    }
  }

  return normal;
}

