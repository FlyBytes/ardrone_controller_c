#ifndef __ARDRONE_HEADER__
#define __ARDRONE_HEADER__

#include <stdio.h>  
#include <winsock2.h>  
#include <iostream>

// C++11头文件
#include <mutex>
#include <thread>

// 本地头文件
#include "NavigationData_Struct.h"
#include "MemoryLibrary.h"
#include "Float2Int_Union.h"

#pragma comment(lib, "ws2_32.lib") 



/**
* ARDrone 无人机控制管理基类
* @author: blyde 
* @update:2015.7.11
**/
class Ardrone
{
public:
	// construction
	Ardrone( char*);
	~Ardrone( void);

public:
	// basic control
	// 根据函数名即可理解函数功能

	void takeoff();
	void land();
	void hover();

	void flyUp();
	void flyDown();
	void flyForward();
	void flyBack();
	void flyLeft();
	void flyRight();

	// 转向函数待实现
	// void turnLeft();	
	// void turnRight();

	// set Ardrone's speed
	void setSpeed( int);	

public:
	// get next sequence number, use C++11's mutex
	int	nextSeq();

	// get current sequence number
	int	getCurrentSeq(){ return this->seq_;}

	// get last sequence number(not use)
	int	getLastSeq(){ return this->seq_ - 1;}

	// get last AT command 
	char* getLastCmd(){ return this->last_at_cmd_;}

	// receive navigation data
	int	receiveNavData();

	// parse navigation data
	int	parseNavData( MemoryLibrary::Buffer&);

public: 
	// send AT*COMWDG command
	int		send_wakeup_cmd();

	// send AT*REF command
	int		send_cmd( const char*);	

	// send AT*PCMD command
	int		send_pcmd( int, float, float, float, float);

	// send all of AT command
	int		send_at_cmd( char*);							

private:
	// initialize command
	int	initArdrone();

	// initialize socket
	int	initSocketAddr();	
	
private:
	// 无人机网络参数
	SOCKET		socketat_;
	sockaddr_in atSin_;
	bool		isInit_;		// 无人机初始化成功标志

	// 无人机数据包参数
	int			seq_;			// sequence of data packet 
	char*		last_at_cmd_;	// save the last command
	std::mutex	mtx_;			// mutex for critical section
	
private:
	// 无人机信息参数
	char*		name_;			// ardrone's name
	float		speed_;			// fly speed
	NavData*	navData_;		// ardrone's navdata 
};

#endif // __ARDRONE_HEADER__
