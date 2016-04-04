#ifndef __NAVDATA_HEADER__
#define __NAVDATA_HEADER__
 
/**
* ARDrone Navdata Struct
* 
* @解释：参考开发手册P42，允许进一步加入新的成员变量
* @author: blyde 
* @update:2015.7.11
**/
struct NavData
{
	int32_t header;			// 
	int32_t state;			// 
	int32_t sequence;		// 
	int32_t visionDefined;
	int16_t tag;
	int16_t size;
	int32_t ctrlState;
	int32_t batteryLevel;
	int32_t pitch;			// 
	int32_t roll;			// 
	int32_t yaw;			// 
	int32_t altitude;
	int32_t vx;				// 
	int32_t vy;				// 
	int32_t vz;				// 
};

#endif // __NAVDATA_HEADER__
