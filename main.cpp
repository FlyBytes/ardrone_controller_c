/**
* 必读
* 函数注释规范：
*
/**
* 复杂函数注释介绍
* 空行
* @解释：
* @param: 参数1 % 注释
* @param: 参数2 % 注释
* 空行
* @return: 返回值 % 写出在什么情况下，返回什么值
* @author: 函数作者
* @updated: 修改者1
* @updated: 修改者2
* @update: 最近一次修改时间
**/

#include "Ardrone.h"

/**
* 唤醒无人机连接的线程
* 一直让基站与无人机保持连接状态
* 
* @param: ardrone: Ardrone*
* @author: blyde
* @update: 2015.7.11
**/
void wakeUpThread(Ardrone* ardrone)
{
	int ret = ardrone->send_wakeup_cmd();
	if ( ret == 0)
		std::cout<<"found an error in weakUpThread"<<std::endl;
}

/**
* 获取导航数据的线程
* 开启线程后，让基站接收无人机发送的导航数据报
* 
* @param: ardrone: Ardrone*
* @author: blyde
* @update: 2015.7.11
**/
void navDataThread(Ardrone* ardrone)
{
	int ret = ardrone->receiveNavData();
	if ( ret == 0)
		std::cout<<"found an error in navDataThread"<<std::endl;
}

/**
* 主函数：声明定义无人机操作控制变量
* 开启获取导航数据线程，开启发送唤醒指令线程，保持主线程
* 
* @param: ardrone: Ardrone*
* @author: blyde
* @update: 2015.7.11
**/
int main(int argc, char* argv[])
{
	Ardrone* ardrone = new Ardrone("myardrone");

	// C++11 声明导航数据的线程
	std::thread wakeThread(wakeUpThread, ardrone);
	std::thread navThread(navDataThread, ardrone);

	// 空循环：保持工作线程运行
	while (true)
	{
		Sleep(1000);
		std::cout<<"Main thread living..."<<std::endl;
	}
	delete ardrone;
	return 0;
}
