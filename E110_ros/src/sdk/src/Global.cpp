#include "../include/Global.h"
#include "../include/usererror.h"
double getAngleWithViewpoint(float r1, float r2, double included_angle)
{
	return atan2(r2 * sin(included_angle), r1 - r2 * cos(included_angle));
}
int int_cmper(const void *p1, const void *p2)
{
	int *d1 = (int *)p1;
	int *d2 = (int *)p2;
	return *d1 - *d2;
}

unsigned int stm32crc(unsigned int *ptr, unsigned int len)
{
	unsigned int xbit, data;
	unsigned int crc32 = 0xFFFFFFFF;
	const unsigned int polynomial = 0x04c11db7;

	for (unsigned int i = 0; i < len; i++)
	{
		xbit = 1 << 31;
		data = ptr[i];
		for (unsigned int bits = 0; bits < 32; bits++)
		{
			if (crc32 & 0x80000000)
			{
				crc32 <<= 1;
				crc32 ^= polynomial;
			}
			else
				crc32 <<= 1;

			if (data & xbit)
				crc32 ^= polynomial;

			xbit >>= 1;
		}
	}
	return crc32;
}
std::string BaseAPI::stringfilter(char *str, int num)
{
	int index = 0;
	for (int i = 0; i < num; i++)
	{
		if ((str[i] >= 45 && str[i] <= 58) || (str[i] >= 65 && str[i] <= 90) || (str[i] >= 97 && str[i] <= 126) || str[i] == 32 || str[i] == '_' /*||str[i]==10||str[i]==13*/)
		{
			index++;
		}
		else
		{
			std::string arr = str;
			arr = arr.substr(0, index);
			return arr;
		}
	}
	return "";
}

void DecTimestamp(uint32_t ts, uint32_t *ts2)
{
	timeval tv;
	gettimeofday(&tv, NULL);

	uint32_t sec = tv.tv_sec % 3600;
	if (sec < 5 && ts / 1000 > 3595)
	{
		ts2[0] = (tv.tv_sec / 3600 - 1) * 3600 + ts / 1000;
	}
	else
	{
		ts2[0] = (tv.tv_sec / 3600) * 3600 + ts / 1000;
	}

	ts2[1] = (ts % 1000) * 1000;
}

bool uart_talk(int fd, int n, const char *cmd,
			   int nhdr, const char *hdr_str,
			   int nfetch, char *fetch)
{
	printf("send command : %s\n", cmd);
	write(fd, cmd, n);

	char buf[2048];
	int nr = read(fd, buf, sizeof(buf));
	while (nr < (int)sizeof(buf))
	{
		int n = read(fd, buf + nr, sizeof(buf) - nr);
		if (n > 0)
			nr += n;
	}
	for (int i = 0; i < (int)sizeof(buf) - nhdr - nfetch; i++)
	{
		if (memcmp(buf + i, hdr_str, nhdr) == 0)
		{
			if (nfetch > 0)
			{
				if (strcmp(cmd, "LXVERH") == 0 || strcmp(cmd, "LUUIDH") == 0 || strcmp(cmd, "LTYPEH") == 0)
				{
					memcpy(fetch, buf + i + nhdr, nfetch);
					fetch[nfetch] = 0;
				}
				else
				{
					strcpy(fetch, "OK");
					fetch[3] = 0;
				}
			}
			return true;
		}
		else if (memcmp(buf + i, cmd, n) == 0)
		{
			if (nfetch > 0)
			{
				memcpy(fetch, buf + i + n + 1, 2);
				fetch[2] = 0;
			}
			return true;
		}
		else if (memcmp(buf + i, "unsupport", 9) == 0)
		{
			if (nfetch > 0)
			{
				strcpy(fetch, "unsupport");
				fetch[10] = 0;
			}
			return false;
		}
	}

	printf("read %d bytes, not found %s\n", nr, hdr_str);
	return false;
}

#ifdef _WIN32
void gettimeofday(timeval *tv, void *)
{
	SYSTEMTIME st;
	GetLocalTime(&st);

	tv->tv_sec = time(NULL);
	tv->tv_usec = st.wMilliseconds;
}
#endif

int SystemAPI::GetComList(std::vector<UARTARG> &list)
{
	std::vector<int> port_list;
	bool isOK = false;
	port_list.push_back(230400);
	port_list.push_back(256000);
	port_list.push_back(384000);
	port_list.push_back(460800);
	port_list.push_back(500000);
	port_list.push_back(768000);
	port_list.push_back(921600);
	port_list.push_back(1000000);
	port_list.push_back(1500000);
	std::vector<std::string> portNames = SystemAPI::GetComPort();
	if (portNames.size() <= 0)
		return 0;

	for (unsigned int i = 0; i < portNames.size(); i++)
	{
		UARTARG arg;
		for (unsigned int j = 0; j < port_list.size(); j++)
		{
			int com_speed = port_list.at(j);
			if (GetDevInfoByUART(portNames.at(i).c_str(), com_speed))
			{
				arg.port = com_speed;
				strcpy(arg.portName, portNames.at(i).c_str());
				list.push_back(arg);
				isOK = true;
				break;
			}
		}
	}
	return 0;
}
int GetDevInfoByUART(const char *port_str, int speed)
{
	int zeroNum = 0;
	unsigned int check_size = 4096;
	int hPort = SystemAPI::open_serial_port(port_str, speed);
	if (hPort <= 0)
	{
		return false;
	}

	char cmd[] = "LUUIDH";
	write(hPort, cmd, sizeof(cmd));
	int bOK = false;
	unsigned char *buf = new unsigned char[check_size];
	unsigned long rf = 0;
	while (rf < check_size)
	{
		int tmp = read(hPort, buf + rf, check_size - rf);
		if (tmp > 0)
		{
			rf += tmp;
			// zeroNum = 0;
		}
		else
		{
			zeroNum++;
			msleep(1);
		}

		if (zeroNum > 10)
		{
			// printf("read 0 byte max index break\n");
			break;
		}
	}
	/*if (zeroNum <= 10)
		printf("read max byte break\n");*/

	if (rf > 10)
	{
		for (unsigned int idx = 0; idx < rf - 10; idx++)
		{
			if (memcmp((char *)buf + idx, "PRODUCT SN:", 11) == 0)
			{
				bOK = 1;
				break;
			}
		}
	}
	SystemAPI::closefd(hPort, false);
	delete[] buf;
	return bOK;
}

int SystemAPI::open_serial_port(const char *name, int speed)
{
	return Open_serial_port(name, speed);
}

int SystemAPI::closefd(int __fd, bool isSocket)
{
#ifdef _WIN32
	if (!isSocket)
		CloseHandle((HANDLE)__fd);
	else
		closesocket(__fd);
	return 0;
#elif __linux
	UNUSED(isSocket);
	shutdown(__fd, SHUT_RDWR);
	return close(__fd);
#endif
}
std::vector<std::string> SystemAPI::GetComPort()
{
#ifdef _WIN32
	HKEY hKey;
	std::vector<std::string> comName;
	char portName[256], commName[256];
	// 打开串口注册表对应的键值
	if (ERROR_SUCCESS == ::RegOpenKeyEx(HKEY_LOCAL_MACHINE, "Hardware\\DeviceMap\\SerialComm", NULL, KEY_READ, &hKey))
	{
		int i = 0;
		int mm = 0;
		DWORD dwLong, dwSize;
		while (TRUE)
		{
			dwLong = dwSize = sizeof(portName);
			// 枚举串口
			if (ERROR_NO_MORE_ITEMS == ::RegEnumValue(hKey, i, portName, &dwLong, NULL, NULL, (PUCHAR)commName, &dwSize))
			{
				break;
			}
			comName.push_back(commName);
			i++;
		}
		// 关闭注册表
		RegCloseKey(hKey);
	}
	// 返回串口号
	return comName;
#elif __linux

	std::vector<std::string> comName;
	std::string path = "/dev";
	DIR *dirp;
	struct dirent *dp;

	dirp = opendir(path.c_str());
	if (dirp == NULL)
	{
		printf("opendir %s failed\n", path.c_str());
		return comName;
	}

	while ((dp = readdir(dirp)) != NULL)
	{
		std::string curpath(path);
		if (path.at(path.length() - 1) != '/')
		{
			curpath += std::string("/") += dp->d_name;
		}
		else
		{
			curpath += dp->d_name;
		}
		// 判断是否为文件以及文件后缀名
		if (dp->d_type == DT_CHR)
		{

			if (curpath.find("ttyUSB") != std::string::npos || curpath.find("ttyACM") != std::string::npos)
			{
				// std::cout<<curpath<<std::endl;
				comName.push_back(curpath);
			}
		}
	}
	closedir(dirp);
	return comName;
#endif
}

#ifdef _WIN32
int read(int __fd, void *__buf, int __nbytes)
{
	DWORD nr;
	bool ret = ReadFile((HANDLE)__fd, __buf, __nbytes, &nr, NULL);
	if (ret == false)
		return -1;
	return nr;
}
int write(int __fd, const void *__buf, int __n)
{
	DWORD nr = 0;
	WriteFile((HANDLE)__fd, __buf, __n, &nr, NULL);
	return nr;
}

int Open_serial_port(const char *name, int port)
{
	char path[32];
	sprintf_s(path, 30, "\\\\.\\%s", name);
	// Open the serial port.
	HANDLE hPort = CreateFile(path,
							  GENERIC_READ | GENERIC_WRITE,		  // Access (read-write) mode
							  FILE_SHARE_READ | FILE_SHARE_WRITE, // Share mode
							  NULL,								  // Pointer to the security attribute
							  OPEN_EXISTING,					  // How to open the serial port
							  0,								  // Port attributes
							  NULL);							  // Handle to port with attribute

	if (hPort == NULL || hPort == INVALID_HANDLE_VALUE)
	{
		// MessageBox(0, "can not open port", name, MB_OK);
		return 0;
	}
	DCB PortDCB;
	// Initialize the DCBlength member.
	PortDCB.DCBlength = sizeof(DCB);
	// Get the default port setting information.
	GetCommState(hPort, &PortDCB);

	// Change the DCB structure settings.
	PortDCB.BaudRate = port;	  // 115200;              // Current baud
	PortDCB.fBinary = TRUE;		  // Binary mode; no EOF check
	PortDCB.fParity = TRUE;		  // Enable parity checking
	PortDCB.fOutxCtsFlow = FALSE; // No CTS output flow control
	PortDCB.fOutxDsrFlow = FALSE; // No DSR output flow control
	PortDCB.fDtrControl = DTR_CONTROL_ENABLE;
	// DTR flow control type
	PortDCB.fDsrSensitivity = FALSE;  // DSR sensitivity
	PortDCB.fTXContinueOnXoff = TRUE; // XOFF continues Tx
	PortDCB.fOutX = FALSE;			  // No XON/XOFF out flow control
	PortDCB.fInX = FALSE;			  // No XON/XOFF in flow control
	PortDCB.fErrorChar = FALSE;		  // Disable error replacement
	PortDCB.fNull = FALSE;			  // Disable null stripping
	PortDCB.fRtsControl = RTS_CONTROL_ENABLE;
	// RTS flow control
	PortDCB.fAbortOnError = FALSE; // Do not abort reads/writes on
	// error
	PortDCB.ByteSize = 8;		   // Number of bits/byte, 4-8
	PortDCB.Parity = NOPARITY;	   // 0-4=no,odd,even,mark,space
	PortDCB.StopBits = ONESTOPBIT; // 0,1,2 = 1, 1.5, 2

	// Configure the port according to the specifications of the DCB
	// structure.
	if (!SetCommState(hPort, &PortDCB))
	{
		// MessageBox(0, "Unable to configure the serial port", "error", MB_OK);
		CloseHandle(hPort);
		return NULL;
	}
	// Retrieve the timeout parameters for all read and write operations
	// on the port.
	COMMTIMEOUTS CommTimeouts;
	GetCommTimeouts(hPort, &CommTimeouts);

	// Change the COMMTIMEOUTS structure settings.
	CommTimeouts.ReadIntervalTimeout = MAXDWORD;
	CommTimeouts.ReadTotalTimeoutMultiplier = 0;
	CommTimeouts.ReadTotalTimeoutConstant = 0;
	CommTimeouts.WriteTotalTimeoutMultiplier = 0; // 10;
	CommTimeouts.WriteTotalTimeoutConstant = 0;	  // 1000;

	// Set the timeout parameters for all read and write operations
	// on the port.
	if (!SetCommTimeouts(hPort, &CommTimeouts))
	{
		// Could not set the timeout parameters.
		// MessageBox(0, "Unable to set the timeout parameters", "error", MB_OK);
		CloseHandle(hPort);
		return NULL;
	}

	return (int)hPort;
}
#elif __linux
int Open_serial_port(const char *port, int baud_rate)
{
	int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd < 0)
	{
		printf("\033[1;31m----> Open %s error\033[0m\n", port);
		return -1;
	}

	int ret;
	struct termios attrs;
	tcflush(fd, TCIOFLUSH);

	/* get current attrs */
	ret = tcgetattr(fd, &attrs);
	if (ret < 0)
	{
		printf("get attrs failed");
		return -2;
	}

	/* set speed */
	int speed = B230400;
	// if (baudrate == 115200) speed = B115200;

	ret = cfsetispeed(&attrs, speed);  //[baudrate]);
	ret |= cfsetospeed(&attrs, speed); //[baudrate]);

	/* enable recieve and set as local line */
	attrs.c_cflag |= (CLOCAL | CREAD);

	/* set data bits */
	attrs.c_cflag &= ~CSIZE;
	attrs.c_cflag |= CS8;

	/* set parity */
	if (1)
	{							  // parity == UART_POFF) {
		attrs.c_cflag &= ~PARENB; // disable parity
		attrs.c_iflag &= ~INPCK;
	}
	else
	{
		attrs.c_cflag |= (PARENB | PARODD); // enable parity
		attrs.c_iflag |= INPCK;
		// if(parity == UART_PEVEN) attrs.c_cflag &= ~PARODD;
	}

	/* set stop bits */
	attrs.c_cflag &= ~CSTOPB; // 1 stop bit
	// attrs.c_cflag |= CSTOPB;	// 2 stop bits

	// Disable Hardware flowcontrol
	attrs.c_cflag &= ~CRTSCTS;

	/* set to raw mode, disable echo, signals */
	attrs.c_lflag &= ~(ICANON | ECHO | ECHOE | IEXTEN | ISIG);

	/* set no output process, raw mode */
	attrs.c_oflag &= ~OPOST;
	attrs.c_oflag &= ~(ONLCR | OCRNL);

	/* disable CR map  */
	attrs.c_iflag &= ~(ICRNL | INLCR);
	/* disable software flow control */
	attrs.c_iflag &= ~(IXON | IXOFF | IXANY);

	//	attrs.c_cc[VMIN] = 0;
	//	attrs.c_cc[VTIME] = 10;

	/* flush driver buf */
	tcflush(fd, TCIFLUSH);

	/* update attrs now */
	if (tcsetattr(fd, TCSANOW, &attrs) < 0)
	{
		close(fd);
		printf("tcsetattr err");
		return -3;
	}

	if (change_baud(fd, baud_rate))
	{
		close(fd);
		printf("fail to set baudrate %d", baud_rate);
		return -4;
	}

	return fd;
}
#endif // _WIN32

bool CommunicationAPI::uart_talk(int fd, int n, const char *cmd, int nhdr, const char *hdr_str, int nfetch, char *fetch, int waittime)
{
	SystemAPI::Write(fd, cmd, n);
	char buf[1024];
	int nr = SystemAPI::Read(fd, buf, sizeof(buf));
	int idx = waittime;
	while (nr < (int)sizeof(buf))
	{
		int n = SystemAPI::Read(fd, buf + nr, sizeof(buf) - nr);
		// printf(" fd %d %d \n",n,nr);
		if (n > 0)
		{
			nr += n;
			idx = waittime;
		}
		else if (n == 0)
		{
			idx--;
			msleep(1);
			if (idx == 0)
			{
				// printf("read 0 byte max index break %s\n",cmd);
				break;
			}
		}
	}
	if (idx != 0)
	{
		// printf("read all byte break %s\n",cmd);
	}
	for (unsigned int i = 0; i < sizeof(buf) - nhdr - nfetch; i++)
	{
		if (memcmp(buf + i, hdr_str, nhdr) == 0 && nhdr > 0)
		{
			if (nfetch > 0)
			{
				if (strcmp(cmd, "LUUIDH") == 0 || strcmp(cmd, "LTYPEH") == 0 || strcmp(cmd, "LQAZNH") == 0 || strcmp(cmd, "LQPSTH") == 0 || strcmp(cmd, "LQNPNH") == 0 || strcmp(cmd, "LQOUTH") == 0 || strcmp(cmd, "LQCIRH") == 0 || strcmp(cmd, "LQFIRH") == 0 || strcmp(cmd, "LQSRPMH") == 0 || strcmp(cmd, "LQSMTH") == 0 || strcmp(cmd, "LQDSWH") == 0 || strcmp(cmd, "LQZTPH") == 0 || strcmp(cmd, "LQSAFH") == 0 || strcmp(cmd, "LQZIRH") == 0 || strcmp(cmd, "LQMCCWH") == 0 || strcmp(cmd, "LQBPSH") == 0 || strcmp(cmd, "LQRESH") == 0)
				{
					memcpy(fetch, buf + i + nhdr, nfetch);
					fetch[nfetch] = 0;
				}
				else if (strstr(cmd, "LSRPM") != NULL)
				{
					if (buf[i + nhdr + 1] == 'O' && buf[i + nhdr + 2] == 'K')
					{
						strncpy(fetch, "OK", 2);
						fetch[3] = 0;
					}
					else if (buf[i + nhdr + 1] == 'e' && buf[i + nhdr + 2] == 'r')
					{
						strncpy(fetch, "NG", 2);
						fetch[3] = 0;
					}
				}
				else if (strcmp(cmd, "LXVERH") == 0 || strcmp(cmd, "LVERSH") == 0)
				{
					std::string mcuversion, motorversion, motorhversion;
					std::string str = std::string(&buf[i + nhdr], nfetch);
					int separatorIdx = str.find("\r\n");
					mcuversion = str.substr(0, separatorIdx);
					str = str.substr(separatorIdx + 2);
					int motoridx = str.find("MOTOR VERSION:");
					if (motoridx != std::string::npos)
					{
						separatorIdx = str.find("\r\n");
						motorversion = str.substr(motoridx + 14, separatorIdx - motoridx - 14);
					}
					str = str.substr(separatorIdx + 2);
					int hmotoridx = str.find("MOTOR HVERSION:");
					if (hmotoridx != std::string::npos)
					{
						separatorIdx = str.find("\r\n");
						motorhversion = str.substr(hmotoridx + 15, separatorIdx - hmotoridx - 15);
					}

					sprintf(fetch, "%s|%s|%s", mcuversion.c_str(), motorversion.c_str(), motorhversion.c_str());
				}
				else
				{
					strncpy(fetch, "OK", 2);
					fetch[3] = 0;
				}
			}
			return true;
		}
		else if (memcmp(buf + i, cmd, n) == 0)
		{
			if (nfetch > 0)
			{
				memcpy(fetch, buf + i + n + 1, 2);
				if (buf[i + n + 1] == 'E' && buf[i + n + 2] == 'R')
				{
					fetch[0] = 'N';
					fetch[1] = 'G';
				}
				fetch[2] = 0;
			}
			return true;
		}
		else if (memcmp(buf + i, "unsupport", 9) == 0)
		{
			if (nfetch > 0)
			{
				strcpy(fetch, "unsupport");
				fetch[10] = 0;
			}
			return true;
		}
	}
	printf("read %d bytes, not found %s\n", nr, hdr_str);
	return false;
}
static const uint8_t CrcTable[256] =
	{
		0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
		0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
		0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
		0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
		0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
		0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
		0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
		0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
		0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
		0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
		0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
		0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
		0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
		0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
		0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
		0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
		0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
		0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
		0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
		0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
		0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
		0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8};
uint8_t LD::CalCRC8(uint8_t *p, uint8_t len)
{
	uint8_t crc = 0;
	uint16_t i;
	for (i = 0; i < len; i++)
	{
		crc = CrcTable[(crc ^ *p++) & 0xff];
	}
	return crc;
}

int LD::parse_data(int len, unsigned char *buf, RawDataHdr_LD *dat, int &consume, char *result)
{
	int idx = 0;
	for (idx = 0; idx <= len - 12; idx++)
	{
		if (buf[idx] == 0x54 && buf[idx + 1] == 0x2c)
		{
			int use_len = 0;
			memcpy(&use_len, &buf[idx + 1], 1);
			if (use_len + 3 > len)
				return -1;

			uint8_t tmpcrc = LD::CalCRC8(&buf[idx], use_len + 2);
			uint8_t crc = buf[idx + use_len + 2];
			if (tmpcrc != crc)
			{
				//DEBUG("crc err!");
				consume = idx + 2;
				return 0;
			}

			dat->header = 0x54;
			dat->N = (use_len - 8) / 3;
			memcpy(&dat->speed, &buf[idx + 2], 2);
			memcpy(&dat->start_angle, &buf[idx + 4], 2);
			memcpy(&dat->end_angle, &buf[idx + 6 + (dat->N * 3)], 2);

			// memcpy(&dat->timestamp, &buf[idx + 8 + (dat->N * 3)], 2);

			int diff = dat->end_angle - dat->start_angle;
			float span = diff > 0 ? diff/100.0 : (diff + 36000)/100.0;
			float angle_interval = 1.0 * span / (dat->N - 1);
			for (unsigned int i = 0; i < dat->N; i++)
			{
				memcpy(&dat->points[i].distance, &buf[idx + 6 + 3 * i], 2);
				dat->points[i].confidence = buf[idx + 8 + 3 * i];
				float tmp = (dat->start_angle/100.0 + i * angle_interval);
				dat->points[i].angle = (tmp > 360) ? (tmp - 360) : tmp;
			}
			//DEBUG("%d %d\n",idx,use_len);
			consume = idx + use_len + 3;
			return 1;
		}
		else if (buf[idx] == 0x0d && buf[idx + 1] == 0x0a)
		{
			int tmpidx = 0;
			for (int i = idx - 1; i >= 0; i--)
			{
				if (buf[i] == 13 || buf[i] == 32 || buf[i] == 33 || (buf[i] >= 43 && buf[i] <= 58) || (buf[i] >= 65 && buf[i] <= 90) || buf[i] == 95 || (buf[i] >= 97 && buf[i] <= 122))
				{
					tmpidx++;
				}
				else
				{
					break;
				}
			}
			if (tmpidx > 0)
			{
				memcpy(result, &buf[idx - tmpidx], tmpidx);
				consume = idx + 1;
				return 11;
			}
		}
	}

	if (idx > 1024)
		consume = idx / 2;
	return -1;
}

int LD::whole_data_process(RawDataHdr_LD *raws, int &nfan, RawDataHdr_LD &tmpraw, FrameData_LD &framedata)
{
	if (tmpraw.N > 0)
	{

		for (int i = 0; i < tmpraw.N; i++)
		{
			//DEBUG("%d %f  \n",i,tmpraw.points[i].angle);
			memcpy(&framedata.points[framedata.N], &tmpraw.points[i], sizeof(LidarPoint_LD));
			framedata.N++;
		}
		// DEBUG("%d %d\n",tmpraw.N,framedata.N);
		tmpraw.N = 0;
	}

	for (int i = 0; i < nfan; i++)
	{
		RawDataHdr_LD *raw = &raws[i];
		for (int j = 0; j < raw->N; j++)
		{
			if (framedata.N == 0 || raw->points[j].angle > framedata.points[framedata.N - 1].angle)
			{
				//DEBUG("%d %d %f  %f  %d %d %d\n",i,j,raw->points[j].angle,framedata.points[framedata.N-1].angle,raws[i].N,framedata.N,nfan);
				memcpy(&framedata.points[framedata.N], &raw->points[j], sizeof(LidarPoint_LD));
				framedata.N++;
				continue;
			}
			if (raw->points[j].angle < framedata.points[framedata.N - 1].angle)
			{
				//DEBUG("%d %d  %f  %f %d %d %d\n",i,j,raw->points[j].angle,framedata.points[framedata.N-1].angle,raws[i].N,framedata.N,nfan);
				if (raw->points[j].angle > 1)
				{
					DEBUG("%d %d  %f  %f %d %d %d\n", i, j, raw->points[j].angle, framedata.points[framedata.N - 1].angle, raws[i].N, framedata.N, nfan);
					 framedata.N=0;
					 tmpraw.N=0;
					 nfan=0;
					 continue;
				}

				for (unsigned int k = j; k < raw->N; k++)
				{
					memcpy(&tmpraw.points[tmpraw.N], &raw->points[k], sizeof(LidarPoint_LD));
					tmpraw.N++;
				}
				int tmpidx = 0;
				for (unsigned int k = i + 1; k < nfan; k++)
				{
					raws[tmpidx] = raws[k];
					tmpidx++;
				}
				nfan = tmpidx;
				return 1;
			}
		}
	}
	nfan = 0;
	return 0;
}

// 这里对原始数据处理，如果是0，即未检测到，不计入最小值，平均值以及后续计算
void calculation(std::vector<float> data, float &min, float &max, float &ave, float &sd)
{
	if (data.size() < 2)
		return;
	float sum = 0, tmp = 0, idx = 0;
	max = FLT_MIN;
	min = FLT_MAX;
	for (int j = 0; j < data.size(); j++)
	{
		if (data[j] == 0)
			continue;

		if (data[j] < min)
			min = data[j];
		if (data[j] > max)
			max = data[j];
		sum += data[j];
		idx++;
	}
	ave = sum / idx;
	for (int j = 0; j < idx; j++)
		tmp += pow(data[j] - ave, 2);

	sd = sqrt(tmp / idx);
}

int SystemAPI::Read(int __fd, void *__buf, int __nbytes)
{
#ifdef _WIN32
	DWORD nr;
	bool ret = ReadFile((HANDLE)__fd, __buf, __nbytes, &nr, NULL);
	if (ret == false)
		return -1;
	return nr;
#elif __linux
	return read(__fd, __buf, __nbytes);
#endif
}
int SystemAPI::Write(int __fd, const void *__buf, int __n)
{
#ifdef _WIN32
	DWORD nr = 0;
	WriteFile((HANDLE)__fd, __buf, __n, &nr, NULL);
	return nr;
#elif __linux
	return write(__fd, __buf, __n);
#endif
}