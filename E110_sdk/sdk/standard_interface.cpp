#include "standard_interface.h"
BlueSeaLidarSDK *BlueSeaLidarSDK::m_sdk = new (std::nothrow) BlueSeaLidarSDK();

BlueSeaLidarSDK *BlueSeaLidarSDK ::getInstance()
{
	return m_sdk;
}
void BlueSeaLidarSDK::deleteInstance()
{
	if (m_sdk)
	{
		delete m_sdk;
		m_sdk = NULL;
	}
}

BlueSeaLidarSDK::BlueSeaLidarSDK()
{
	m_idx = 0;
}

BlueSeaLidarSDK::~BlueSeaLidarSDK()
{
}
RunConfig *BlueSeaLidarSDK::getLidar(int ID)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
			return m_lidars.at(i);
	}

	return NULL;
}
int BlueSeaLidarSDK::addLidarByPath(std::string cfg_file_name)
{
	RunConfig *cfg = new RunConfig;
	memset((char *)cfg, 0, sizeof(RunConfig));
	if (readConfig(cfg_file_name.c_str(), cfg->runscript))
	{
		m_idx++;
		cfg->ID = m_idx;
		m_lidars.push_back(cfg);
		return m_idx;
	}
	else
		return 0;
}

bool BlueSeaLidarSDK::delLidarByID(int ID)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			if (m_lidars.at(i)->state == WORK)
			{
				m_lidars.at(i)->state = UNINIT;
				sleep(1);
			}
			m_lidars.erase(m_lidars.begin() + i);
			return true;
		}
	}
	return false;
}

void BlueSeaLidarSDK::setCallBackPtr(int ID, printfMsg ptr)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			m_lidars.at(i)->callback = ptr;
		}
	}
}

bool BlueSeaLidarSDK::connect(int ID)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	int fd = SystemAPI::open_serial_port(lidar->runscript.connectArg, lidar->runscript.connectArg2);
	if (fd <= 0)
	{
		return false;
	}
	lidar->fd = fd;

#ifdef __unix__
	if (pthread_create(&lidar->thread_data, NULL, lidar_thread_proc_uart, lidar) != 0)
		return false;
#elif _WIN32
	if ((int)CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)lidar_thread_proc_uart, lidar, 0, &lidar->thread_data) < 0)
		return false;
#endif

	lidar->state = WORK;
	// 判定数据线程是否正常运行
	int index = 100;
	while (lidar->action < ONLINE && index > 0)
	{
		msleep(100);
		index--;
	}
	if (lidar->action >= ONLINE)
		return true;

	return false;
}

// 释放连接
void BlueSeaLidarSDK::disconnect(int ID)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			m_lidars.at(i)->action = OFFLINE;
		}
	}
}
std::string BlueSeaLidarSDK::getUUID(int ID)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return "";

	lidar->send_len = 6;
	strcpy(lidar->send_cmd, "LUUIDH");
	lidar->action = QUERY;

	int index = 300;
	while (lidar->action != FINISH && index > 0)
	{
		msleep(10);
		index--;
	}
	printf("%s %d %d %s\n",__FUNCTION__,__LINE__,index,BaseAPI::stringfilter(lidar->recv_cmd,lidar->recv_len).c_str());
	if (lidar->action == FINISH)
	{	
		return BaseAPI::stringfilter(lidar->recv_cmd,lidar->recv_len);
	}

	return "";
}
std::string BlueSeaLidarSDK::getModel(int ID)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return "";

	lidar->send_len = 6;
	strcpy(lidar->send_cmd, "LTYPEH");
	lidar->action = QUERY;

	int index = 300;
	while (lidar->action != FINISH && index > 0)
	{
		msleep(10);
		index--;
	}
	printf("%s %d %d %s\n",__FUNCTION__,__LINE__,index,BaseAPI::stringfilter(lidar->recv_cmd,lidar->recv_len).c_str());

	if (lidar->action == FINISH)
	{	
		return BaseAPI::stringfilter(lidar->recv_cmd,lidar->recv_len);
	}

	return "";
}
std::string BlueSeaLidarSDK::hardwareVersion(int ID)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return "";

	lidar->send_len = 6;
	strcpy(lidar->send_cmd, "LVERSH");
	lidar->action = QUERY;

	int index = 300;
	while (lidar->action != FINISH && index > 0)
	{
		msleep(10);
		index--;
	}
	printf("%s %d %d %s\n",__FUNCTION__,__LINE__,index,BaseAPI::stringfilter(lidar->recv_cmd,lidar->recv_len).c_str());

	if (lidar->action == FINISH)
	{	
		return BaseAPI::stringfilter(lidar->recv_cmd,lidar->recv_len);
	}

	return "";
}
std::string BlueSeaLidarSDK::softwareVersion(int ID)
{
	return SDKVERSION;
}

// 启停雷达测距
bool BlueSeaLidarSDK::setWork(int ID, bool isrun)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	lidar->send_len = 6;
	if(isrun)
		strcpy(lidar->send_cmd, "LSTARH");
	else
		strcpy(lidar->send_cmd, "LSTOPH");


	lidar->action = CONTROL;

	int index = 300;
	while (lidar->action != FINISH && index > 0)
	{
		msleep(10);
		index--;
	}
	if (lidar->action == FINISH)
	{
		printf("%s OK\n", lidar->send_cmd);
		return true;
	}
	return false;
}
