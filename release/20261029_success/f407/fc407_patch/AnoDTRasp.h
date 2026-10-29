#ifndef __ANO_DT_RASP_H
#define __ANO_DT_RASP_H
//==引用
#include "SysConfig.h"

//通信方式定义
#define LinkType_ToRasp		0x01
#define LinkType_ToIMU		0x02
//发送数据帧时临时存储数据帧信息的结构体
typedef struct
{
	u8 _isUsed;
	u8 _linkType;			
}__attribute__ ((__packed__)) myTPTv7TxStruct;

typedef struct
{
	u8 head;
	u8 addr;
	u8 ID;
	u8 dataLen;
	u8 dataBuf[256];
	u8 sc1;
	u8 sc2;
}__attribute__ ((__packed__)) myTAnoPTv7Struct;	 

typedef union 
{
	myTAnoPTv7Struct frame;
	u8 rawBytes[sizeof(myTAnoPTv7Struct)];
}__attribute__ ((__packed__)) myTAnoPTv7Frame;

//用于发送的结构体，data保存帧内容，cmd保存发送参数
typedef struct
{
	myTAnoPTv7Frame data;		
	myTPTv7TxStruct cmd; 			
}__attribute__ ((__packed__)) myTPTv7TxFrame;

typedef struct
{
	u8 getRosDataFlag;//标志位
	s16 tacc[3];//t265加速度
	s16 tloc[3];//t265位置
	s16 loc[3];//激光雷达建图定位
	
    s16 acc[3];//ros控制无人机速度
    s16 yaw;//ros控制无人机平航
}ROS_DATA_ST;
extern ROS_DATA_ST rosData;


void AnoDTRaspRunTask1Ms(void);
void AnoDTRaspRecvOneByte(u8 data);



void AnoPTv7SendALT(s16 alt);
void AnoPTv7SendATT(s16 rol_x100,s16 pit_x100,s16 yaw_x100);
void AnoPTv7SendCTRL(s16 rol,s16 pit,s16 thr,s16 yaw_dps,s16 vel_x,s16 vel_y,s16 vel_z);
void AnoPTv7SendVEL(s16 vel_x,s16 vel_y,s16 vel_z);
void AnoPTv7SendGPS(s32 GPS_LNG,s32 GPS_LAT,s32 GPS_ALT);
void AnoPTv7SendMAG(s16 mag_x,s16 mag_y,s16 mag_z,s16 tmp);
void AnoPTv7SendUserData(u8 d1,u8 d2,u8 d3,s16 d4,s16 d5,s16 d6);
void AnoPTv7SendRouteData(u8 d1,u8 d2);
void AnoPTv7SendIMU(s16 accx,s16 accy,s16 accz,s16 gyr_x,s16 gyr_y,s16 gyr_z,s16 v0,s16 v1,s16 v2,s16 v3);
void AnoPTv7SendRC(u16 ch_1,u16 ch_2,u16 ch_3,u16 ch_4,u16 ch_5,u16 ch_6,u16 ch_7,u16 ch_8,u16 ch_9,u16 ch_10);


#endif
