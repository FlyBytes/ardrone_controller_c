#include "Ardrone.h"
#include <assert.h>

/**
Author: MAPGPS at
	http://www.ardrone-flyers.com/forum/viewforum.php?f=8
	http://www.rcgroups.com/forums/showthread.php?t=1335257
	https://projects.ardrone.org/projects/ardrone-api/boards
	http://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1025
	http://bbs.5imx.com/bbs/viewthread.php?tid=415063
Initial: 2010.09.20
Updated: 2011.03.03

// Define masks for ARDrone state
// 31                                                             0
//  x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x -> state
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | |
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | FLY MASK : (0) ardrone is landed, (1) ardrone is flying
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | VIDEO MASK : (0) video disable, (1) video enable
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | | VISION MASK : (0) vision disable, (1) vision enable
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | CONTROL ALGO : (0) euler angles control, (1) angular speed control
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active
//  | | | | | | | | | | | | | | | | | | | | | | | | | | USER feedback : Start button state
//  | | | | | | | | | | | | | | | | | | | | | | | | | Control command ACK : (0) None, (1) one received
//  | | | | | | | | | | | | | | | | | | | | | | | | Camera enable : (0) Camera enable, (1) camera disable
//  | | | | | | | | | | | | | | | | | | | | | | | Travelling enable : (0) disable, (1) enable
//  | | | | | | | | | | | | | | | | | | | | | | USB key : (0) usb key not ready, (1) usb key ready
//  | | | | | | | | | | | | | | | | | | | | | Navdata demo : (0) All navdata, (1) only navdata demo
//  | | | | | | | | | | | | | | | | | | | | Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent
//  | | | | | | | | | | | | | | | | | | | Motors status : (0) Ok, (1) Motors Com is down
//  | | | | | | | | | | | | | | | | | | Communication Lost : (1) com problem, (0) Com is ok
//  | | | | | | | | | | | | | | | | |
//  | | | | | | | | | | | | | | | | VBat low : (1) too low, (0) Ok
//  | | | | | | | | | | | | | | | User Emergency Landing : (1) User EL is ON, (0) User EL is OFF
//  | | | | | | | | | | | | | | Timer elapsed : (1) elapsed, (0) not elapsed
//  | | | | | | | | | | | | | Magnetometer calibration state : (0) Ok, no calibration needed, (1) not ok, calibration needed
//  | | | | | | | | | | | | Angles : (0) Ok, (1) out of range
//  | | | | | | | | | | | WIND MASK: (0) ok, (1) Too much wind
//  | | | | | | | | | | Ultrasonic sensor : (0) Ok, (1) deaf
//  | | | | | | | | | Cutout system detection : (0) Not detected, (1) detected
//  | | | | | | | | PIC Version number OK : (0) a bad version number, (1) version number is OK
//  | | | | | | | ATCodec thread ON : (0) thread OFF (1) thread ON
//  | | | | | | Navdata thread ON : (0) thread OFF (1) thread ON
//  | | | | | Video thread ON : (0) thread OFF (1) thread ON
//  | | | | Acquisition thread ON : (0) thread OFF (1) thread ON
//  | | | CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled // Check frequency of control loop
//  | | ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good // Check frequency of uart2 dsr (com with adc)
//  | Communication Watchdog : (1) com problem, (0) Com is ok // Check if we have an active connection with a client
//  Emergency landing : (0) no emergency, (1) emergency

AT*REF=<sequence>,<UI>
AT*PCMD=<sequence>,<enable>,<roll>,<pitch>,<gaz>,<yaw>
	(float)0.05 = (int)1028443341		(float)-0.05 = (int)-1119040307
	(float)0.1  = (int)1036831949		(float)-0.1  = (int)-1110651699
	(float)0.2  = (int)1045220557		(float)-0.2  = (int)-1102263091
	(float)0.5  = (int)1056964608		(float)-0.5  = (int)-1090519040
AT*ANIM=<sequence>,<animation>,<duration>
AT*CONFIG=<sequence>,\"<name>\",\"<value>\"

* Thank for sharing!!!
**/

///////////////////////////////////////////////////////////////////////////////////

/**
* 常量定义
**/
const int  NAVDATA_PORT = 5554;			// the port of get navigation data from my ARDrone
const int  VIDEO_PORT   = 5555;			// the port of get video data from my ARDrone
const int  AT_PORT      = 5556;			// the port of send AT Commands from my computer

const int INTERVAL		= 100;			// interval of two AT command is 100ms
const char* ARDRONE_IP	= "192.168.1.1";// the ip address of my ARDrone

const int C_OK			= 1;
const int C_ERROR		= 0;

// 控制指令格式常量
const char* TAKEOFF_FORMAT	= "AT*REF=%d,290718208\r";
const char* LAND_FORMAT		= "AT*REF=%d,290717696\r";


/**
 * 浮点型数据转化为32位表示的int型数据
 * 使用联合体实现float 转化int
 *
 * @解释：根据《计算计组成与结构》课程中浮点型表示方法，
 * 浮点型数据在32位电脑中存储，
 * 为保证不失真，把浮点型存储转化为存储结构相同的int型数据
 * 
 * @param: f:float
 * @author: yikang.zhou
 * @update:  2015.2.7
 */
int	float2Int(float f)
{
	Float2Int buff;
	buff.fBuff = f;
	return buff.iBuff;
}

/////////////////////////Ardrone 类的实现

/**
* 构造函数：初始化无人机连接参数与类的成员变量
* 
* @param1: name: char* % 为无人机命名
* @author: blyde
* @update: 2015.7.11
**/
Ardrone::Ardrone( char* name)
{
	this->isInit_ = false;
	int ret = initSocketAddr();
	if ( ret == C_ERROR)
	{
		std::cout << "ARDrone initialized failed!!!" << std::endl;
		return;
	}
	this->name_		= name;
	this->speed_	= 0.1f;
	this->seq_		= 1;
	this->navData_	= new NavData();
	this->isInit_	= true;

	std::cout << "ARDrone initialized" << std::endl;
	std::cout << "IP:" << ARDRONE_IP << "Port:" << AT_PORT << std::endl;
}

/**
* initialize socket address 
* return C_ERROR when WSAStartup funcation return 1
* return C_OK when initialized successfully
* 
* @author: blyde
* @update: 2015.7.11
**/
int Ardrone::initSocketAddr()
{
	// 在MS-OS下用WSAStartup 启动Ws2_32.lib
	WORD socketVersion = MAKEWORD( 2, 2);
	WSADATA wsaData;
	int ret = WSAStartup( socketVersion, &wsaData);
	if ( ret == 1)	// 当启动失败时，返回C_ERROR
		return C_ERROR;

	socketat_				= socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	atSin_.sin_family		= AF_INET;
	atSin_.sin_port			= htons( AT_PORT);
	atSin_.sin_addr.s_addr	= inet_addr( ARDRONE_IP);
	return C_OK;
}

/** 
* 析构函数
* 
* @author: blyde
* @update: 2015.7.11
**/
Ardrone::~Ardrone()
{
	delete[] last_at_cmd_;
	delete[] name_;
	delete navData_;
	WSACleanup();				// 释放Winsock库
	closesocket( socketat_);	// 关闭SOCKET
}

/**
* initialize Ardrone(not use)
* 设置最大飞行高度，设置超声波频率等无人机参数预设
* 
* @author: blyde
* @update: 2015.7.11
*/
int Ardrone::initArdrone()
{
	char cmd[1024];
	// 设置最大高度
	sprintf_s( cmd, "AT*CONFIG=%d,\"control:altitude_max\",\"2000\"\r", nextSeq());
	assert( send_at_cmd(cmd));
	Sleep( INTERVAL);

	// 设置超声波频率
	sprintf_s( cmd, "AT*CONFIG=%d,\"pic:ultrasound_freq\",\"8\"\r", nextSeq());
	assert( send_at_cmd( cmd));
	Sleep( INTERVAL);
	
	//flat trim
	sprintf_s( cmd, "AT*FTRIM=%d\r", nextSeq());
	assert( send_at_cmd(cmd));
	Sleep( INTERVAL);
	return C_OK;
}

/**
* 飞行控制函数实现部分
**/

// 起飞
void Ardrone::takeoff()
{
	assert( send_cmd(TAKEOFF_FORMAT));
	printf("take off\n");
}

// 降落
void Ardrone::land()
{
	assert( send_cmd( LAND_FORMAT));
	printf("land\n");
}

// 悬停
void Ardrone::hover()
{
	assert( send_pcmd( 0, 0, 0, 0, 0));
}

// 上升
void Ardrone::flyUp()
{
	assert( send_pcmd( 1, 0, 0, speed_, 0));
	printf("goingUp\n");
}

// 下降
void Ardrone::flyDown()
{
	assert( send_pcmd( 1, 0, 0, -speed_, 0));
	printf("goingDown\n");
}

// 向前飞
void Ardrone::flyForward()
{
	assert( send_pcmd( 1, 0, -speed_, 0, 0));
}

// 向后飞
void Ardrone::flyBack()
{
	assert( send_pcmd( 1, 0, speed_, 0, 0));
}

// 向左飞
void Ardrone::flyLeft()
{
	assert( send_pcmd( 1, -speed_, 0, 0, 0));
}

// 向右飞
void Ardrone::flyRight()
{
	assert( send_pcmd( 1, speed_, 0, 0, 0));
}

/*
void Ardrone::turnLeft()
{
	assert(send_pcmd(1, 0, 0, 0, -speed_));
	printf("turn Left\n");
	outfile << "Turn left" << endl;
}

void Ardrone::turnRight()
{
	assert(send_pcmd(1, 0, 0, 0, speed_));
	printf("turnRight\n");
	outfile << "Turn right" << endl;
}
*/

// 设置飞行速度
void Ardrone::setSpeed(int mul)
{
	this->speed_ = mul * 0.1f;
}

/**
* get next sequence number, use C++11's mutex
* 
* @author: blyde
* @update: 2015.7.11
**/
int Ardrone::nextSeq()
{
	// 互斥锁:保证数据包编号不重复
	this->mtx_.lock();
	seq_ += 1;
	this->mtx_.unlock();
	return seq_;
}

/**
* send AT*PCMD command
* 发送飞行控制指令，指令规则请翻阅开发手册
* 
* @param: enable % 能否控制
* @param: roll % 我忘了，帮我补上吧，嘻嘻
* @param: pitch % 
* @param: gaz % 
* @param: yaw % 
*
* @author: blyde
* @update: 2015.7.11
**/
int Ardrone::send_pcmd(int enable, float roll, float pitch, float gaz, float yaw)
{
	char cmd[100]; 
	sprintf_s(cmd, "AT*PCMD=%d,%d,%d,%d,%d,%d\r", nextSeq(), 
					enable, float2Int(roll), float2Int(pitch), float2Int(gaz), float2Int(yaw));
	int ret = send_at_cmd( cmd);
	return ret;
}

/**
* send AT*REF command
* 发送AT*REF控制指令
* 
* @param: cmdFormat % 指令格式
* @author: blyde
* @update: 2015.7.11
**/
int Ardrone::send_cmd( const char* cmdFormat)
{
	char cmd[100];
	sprintf_s( cmd, cmdFormat, nextSeq());
	int ret = send_at_cmd( cmd);
	return ret;
}

/**
* send all of AT command
* 发送所有的AT指令
* 
* @param: cmd % 成型的指令
* @author: blyde
* @update: 2015.7.11
**/
int Ardrone::send_at_cmd(char* cmd)
{
	// 互斥锁:保证一个时刻只有一个数据包被发送
	this->mtx_.lock();
	this->last_at_cmd_ = cmd;

	int result = sendto(this->socketat_, cmd, strlen(cmd), 0, (sockaddr *)&atSin_, sizeof(atSin_));
	if (result == SOCKET_ERROR)
		return C_ERROR;

	printf_s("AT command: %s\n", cmd);
	this->mtx_.unlock();
	return C_OK;
}

/**
* send AT*COMWDG command
* 发送唤醒连接线程函数
* 
* @解释：根据开发文档说明，前后两个指令间时间间隔低于2s,因此需要发送AT*COMWDG来维持基站与无人机的连接
* @author: blyde
* @update: 2015.7.11
**/
int Ardrone::send_wakeup_cmd()
{
	char cmd[100];
	int delay = 0;
	int ret = 0;
	if( !isInit_)
		return C_ERROR;
	std::cout<<"__wakeupThread start "<<std::endl;
	while ( true)
	{
		// keep Ardrone wake up
		Sleep(40);
		send_at_cmd(last_at_cmd_);
		delay++;
		if (delay >= 4)
		{
			delay = 0;
			sprintf_s( cmd, "AT*COMWDG=%d\r", nextSeq());
			ret = send_at_cmd( cmd);
			if ( ret == C_ERROR)
			{
				std::cout<<"__send_at_cmd error"<<std::endl;
				return C_ERROR;
			}
		}
	}
}

/**
* receive navigation data
* 接收导航数据线程函数
* 
* @解释：根据开发手册P41，需要完成类似于三次握手协议，先发一个激活数据报，
* 然后配置无人机，才能让无人机向基站发送数据报
* 可以使用WireShark2 检测数据报是否发到本机上
* @author: blyde
* @update: 2015.7.11
**/
int Ardrone::receiveNavData()
{
	if( !isInit_)
		return C_ERROR;

	SOCKET socketNav = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	sockaddr_in	navSin;
	navSin.sin_family = AF_INET;
	navSin.sin_port = htons(NAVDATA_PORT);
	navSin.sin_addr.s_addr = inet_addr(ARDRONE_IP);
	int lenNavSin = sizeof(navSin);

	// 激活指令发往NAVDATA_PORT 端口
	const char trigger[4] = { 0x01, 0x00, 0x00, 0x00 };
	int ret = sendto(socketNav, trigger, strlen(trigger), 0, (sockaddr *)&navSin, lenNavSin);
	
	if (ret == SOCKET_ERROR)
	{
		std::cout<<"__send trigger flag error"<<std::endl;
		return C_ERROR;
	}

	printf_s("Sent trigger flag to UDP port : %d \n", NAVDATA_PORT);

	// 配置指令发往AT_PORT 端口
	char initCmd[100];
	sprintf_s(initCmd, "AT*CONFIG=%d,\"general:navdata_demo\",\"TRUE\"\r", nextSeq());
	ret = send_at_cmd(initCmd);
	if ( ret == C_ERROR)
	{
		std::cout<<"__send initCmd error"<<std::endl;
		return C_ERROR;
	}
	// 接收数据包
	MemoryLibrary::Buffer navDataBuffer;	// 二进制数据缓冲区
	char recv[1024] = { 0 };				// 数据包接收数组
	int lenRecv = 0;
	int delay = 0;
	
	std::cout<<"__receiveNavData start "<<std::endl;
	while ( true)
	{
		// 获取导航数据包
		lenRecv = recvfrom(socketNav, recv, 1024, 0, (struct sockaddr*)&navSin, &lenNavSin);
		delay++;
		if (delay >= 5)
		{
			delay = 0;
			printf("received %d bytes\n", lenRecv);
			// 分析数据包，得到导航数据
			navDataBuffer.Set(recv, lenRecv);
			ret = parseNavData(navDataBuffer);
			if ( ret == C_ERROR)
			{
				std::cout<<"__parseNavData() error"<<std::endl;
				return C_ERROR;
			}
		}
	}
}

/**
* parse navigation data 
* 分析导航数据
* 
* @解释：根据开发手册P39，P40,P42
* @author: blyde
* @update: 2015.7.11
**/
int Ardrone::parseNavData(MemoryLibrary::Buffer& buffer)
{
	int offset = 0;
	int header = buffer.MakeValueFromOffset<int32_t>(offset);
	if (header != 0x55667788)
	{
		std::cout << "NavigationDataReceiver FAIL, because the header != 0x55667788\n";
		return C_ERROR;
	}

	offset = 0;
	navData_->header = buffer.MakeValueFromOffset<int32_t>(offset);
	offset += 4;
	navData_->state = buffer.MakeValueFromOffset<int32_t>(offset);
	offset += 4;
	navData_->sequence = buffer.MakeValueFromOffset<int32_t>(offset);
	offset += 4;
	navData_->visionDefined = buffer.MakeValueFromOffset<int32_t>(offset);
	offset += 4;
	navData_->tag = buffer.MakeValueFromOffset<int16_t>(offset);
	offset += 2;
	navData_->size = buffer.MakeValueFromOffset<int16_t>(offset);

	offset += 2;
	navData_->ctrlState = buffer.MakeValueFromOffset<int32_t>(offset);
	offset += 4;

	// 测试
	int batteryLevel = buffer.MakeValueFromOffset<int32_t>(offset);
	navData_->batteryLevel = batteryLevel;
	
	// 测试数据处理结果
	std::cout<< "Testing parseNavData() batteryLevel:"<< batteryLevel<< std::endl;
	std::cout<< "Testing parseNavData() navData_->batteryLevel:"<< navData_->batteryLevel<<std::endl;

	offset += 4;
	navData_->altitude = buffer.MakeValueFromOffset<int32_t>(offset);
	offset += 4;

	navData_->pitch = buffer.MakeValueFromOffset<int32_t>(offset);
	offset += 4;
	navData_->roll = buffer.MakeValueFromOffset<int32_t>(offset);
	offset += 4;
	navData_->yaw = buffer.MakeValueFromOffset<int32_t>(offset);
	offset += 4;

	navData_->vx = buffer.MakeValueFromOffset<int32_t>(offset);
	offset += 4;
	navData_->vy = buffer.MakeValueFromOffset<int32_t>(offset);
	offset += 4;
	navData_->vz = buffer.MakeValueFromOffset<int32_t>(offset);
	offset += 4;
	return C_OK;
}
