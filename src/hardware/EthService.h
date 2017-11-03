/*
 * EthService.h
 *                                       EtherCAT通信数据结构
 *  Created on: 2015年12月16日
 *      Author: root
 */


#ifndef ETHSERVICE_H_
#define ETHSERVICE_H_

#ifndef PANOSONIC_DRIVER
#define PANOSONIC_DRIVER
#endif

/*
#ifndef COOL_DRIVER
#define COOL_DRIVER
#endif
*/

/*
#ifndef ZZ_DRIVER
#define ZZ_DRIVER
#endif
*/

/*
#ifndef I700_DRIVER
#define I700_DRIVER
#endif
*/

#ifndef RUNNING_MOD
#define RUNNING_MOD
#endif

//#include <rtai.h>
//#include <rtai_sched.h>
//#include <rtai_lxrt.h>
//#include <rtai_sem.h>
//#include <rtai_rwl.h>
//#include <rtai_posix.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cstring>
using namespace std;
#include <stdint.h>
#include <errno.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include	<pthread.h>
#include <semaphore.h>
//#include "global.h"


#define 	ETHERNET_MAX_FRAME_LEN 1514
#define 	ETHERNET_FRAME_TYPE_ECAT 0x88A4
#define 	PDO_CYCL_TIME 2000
#define 	PDO_CYCL_TIME_NS 2000000
#define 	SLAVE_CNT 6
#define 	MOTOR_CNT 6
#define 	IO_CNT 0
#define 	POSITIONCMD 2000
#define 	VELOCITY_CMD 120*2147483648/480000
#define 	ACCELERATION_CMD 1000*2147483648/480000
#define EGEAR_RATIO 0.125
#define 	CYCLE_SCALING 0.002746582//360/131072.0

#pragma pack(1)

	typedef struct MAC_ADRR
	{
		uint8_t Adrr[6];
	}MAC_ADRR,*PMAC_ADRR;
	#define MAC_ADRR_LEN   sizeof(struct MAC_ADRR)


	////////////////////////////////////////////////////////////////////以太网数据帧头数据结构定义
	typedef struct TETHERNET_HEADER
	{
		MAC_ADRR Destination;
		MAC_ADRR Source;
		uint16_t FrameType;
	}TETHERNET_HEADER,*PTETHERNET_HEADER;
#define TETHERNET_HEADER_LEN   sizeof(struct TETHERNET_HEADER)


	///////////////////////////////////////////////////////////////////////ETHERCAT数据头定义
	typedef struct ETYPE_88A4_HEADER
	{
		uint16_t Length        :11;
		uint16_t Reserved    :1;
		uint16_t Type            :4;
	}ETYPE_88A4_HEADER,*PETYPE_88A4_HEADER;
#define ETYPE_88A4_HEADER_LEN   sizeof(struct ETYPE_88A4_HEADER)

	///////////////////////////////////////////////////////////////////////////EtherCAT frame
	typedef struct TETHERNET_88A4_HEADER
	{
		TETHERNET_HEADER Ether;
		ETYPE_88A4_HEADER E88A4;
	}TETHERNET_88A4_HEADER,*PTETHERNET_88A4_HEADER;
#define TETHERNET_88A4_HEADER_LEN   sizeof(struct TETHERNET_88A4_HEADER)

	//////////////////////////////////////////////////////////////////////ECAT命令数据结构
	typedef struct TETYPE_ECAT_HEADER
	{
		uint8_t  cmd;////寻址方式
		uint8_t  idx;/////帧索引
		union
		{
			struct
			{
				uint16_t slAdr;////从站地址
				uint16_t ofsAdr;////数据地址
			};
			uint32_t lgAdr;///逻辑地址，不需要从站编号
		};
		uint16_t length   :11;
		uint16_t res   :4;
		uint16_t next   :1;
		uint16_t irq;
	}TETYPE_ECAT_HEADER,*PTETYPE_ECAT_HEADER;
#define TETYPE_ECAT_HEADER_LEN   sizeof(struct TETYPE_ECAT_HEADER)

	///////////////////////////////////////////////////////////////////////////子报文 MAX Frame
	typedef struct ECAT_MAX_FRAME
	{
		TETYPE_ECAT_HEADER ECATHeader;
		uint8_t Data[ETHERNET_MAX_FRAME_LEN-TETHERNET_88A4_HEADER_LEN-TETYPE_ECAT_HEADER_LEN];
	}ECAT_MAX_FRAME,*PECAT_MAX_FRAME;

	///////////////////////////////////////////////////////////////////////////EtherCAT MAX Frame
	typedef struct TETHERNET_88A4_MAX_FRAME
	{
		//TETHERNET_HEADER Ether;
		ETYPE_88A4_HEADER E88A4;
		uint8_t Data[ETHERNET_MAX_FRAME_LEN-TETHERNET_88A4_HEADER_LEN];
	}TETHERNET_88A4_MAX_FRAME,*PTETHERNET_88A4_MAX_FRAME;

	typedef enum
	{///寻址方式
		NOP=0,
		APRD=1,
		APWR=2,
		APRW=3,
		FPRD=4,
		FPWR=5,
		FPRW=6,
		BRD=7,
		BWR=8,
		BRW=9,
		LRD=10,
		LWR=11,
		LRW=12,
		ARMW=13,
		FRMW=14
	}EC_CMD;

	typedef struct SYNCMANAGE////SM 配置
	{
		uint16_t m_nPhyStart;
		uint16_t m_nLength;
		uint8_t m_cContrl;
		uint8_t m_cStatus;
		uint8_t m_ncActive;
		uint8_t m_cPdiContrl;
	}SYNCMANAGE,*PSYNCMANAGE;

	typedef struct FMMU ////FMMU配置
		{
		uint32_t m_nLgstartAdr;
		uint16_t m_nLength;
		uint8_t m_nLgStartBit    :3;
		uint8_t res1    :5;
		uint8_t m_nLgStopBit    :3;
		uint8_t res2    :5;
		uint16_t m_nPhyStartAdr;
		uint8_t m_nPhyStartBit    :3;
		uint8_t res3    :5;
		uint8_t m_nType;
		uint8_t m_nActive;
		uint16_t res4;
		uint8_t res5;
		}FMMU,*PFMMU;


/////////////////////////////////////////////////////////////////////Mail结构
typedef struct MAIL_HEADER
{
	uint16_t Length;
	uint16_t Addr;
	uint8_t res;
	uint8_t  Type:    4;
	uint8_t  Counter:    4;
}MAIL_HEADER,*PMAIL_HEADER;
#define MAIL_HEADER_LEN   sizeof(struct MAIL_HEADER)

typedef struct COE_HEADER
{
	uint16_t PdoNo   :   9;
	uint16_t res    :   3;
	uint16_t Type    :   4;
}COE_HEADER,*PCOE_HEADER;
#define COE_HEADER_LEN   sizeof(struct COE_HEADER)

typedef struct SDO_HEADER
{
	uint8_t    SodCmd;
	uint16_t    Index;
	uint8_t    Sub;
	uint32_t    Data;
	//uint16_t    Data2;
}SDO_HEADER,*PSDO_HEADER;
#define SDO_HEADER_LEN   sizeof(struct SDO_HEADER)

typedef struct LSDO_HEADER
{
	uint8_t    SodCmd;
	uint16_t    Index;
	uint8_t    Sub;
	uint32_t    Len;
	uint64_t    MailData;
}LSDO_HEADER,*PLSDO_HEADER;
#define LSDO_HEADER_LEN   sizeof(struct LSDO_HEADER)

typedef struct COE_SDO_FRAME
{
	MAIL_HEADER   MailHeader;
	COE_HEADER    CoeHeader;
	SDO_HEADER    SdoHeader;
}COE_SDO_FRAME,*PCOE_SDO_FRAME;
#define COE_SDO_FRAME_LEN   sizeof(struct COE_SDO_FRAME)

typedef struct COE_LSDO_FRAME
{
	MAIL_HEADER   MailHeader;
	COE_HEADER    CoeHeader;
	LSDO_HEADER    SdoHeader;
}COE_LSDO_FRAME,*PCOE_LSDO_FRAME;
#define COE_LSDO_FRAME_LEN   sizeof(struct COE_LSDO_FRAME)

#ifdef I700_DRIVER
typedef struct RPDO_MAPPING
{
	uint16_t CtrlWord;
	uint16_t LenzeCtrl;
	uint8_t Modes;
	short int TorqueOffset;
	int TargetPos;
	int VelocityOffset;
	short int Sp_StartValue;
	uint16_t TouchProbeFunc;
	uint16_t PosTorqueLimit;
	uint16_t NegTorqueLimit;
	int UpperSpeed;
	int LowerSpeed;
}RPDO_MAPPING,*PRPDO_MAPPING;
#define RPDO_MAPPING_LEN   sizeof(struct RPDO_MAPPING)

typedef struct TPDO_MAPPING
{
	uint16_t StatusWord;
	uint16_t LenzStatusWord;
	uint8_t ModesDisplay;
	uint16_t ErroCode;
	int ActualVelocity;
	short int ActualTorque;
	int ActualPosition;
	int FollowingErroValue;
	uint16_t Tou1;
	int Tou2;
	int Tou3;
	int Tou4;
	int Tou5;
	uint16_t LenzStatusWord2;
	uint32_t DigitalIn;
}TPDO_MAPPING,*PTRPDO_MAPPING;
#define TPDO_MAPPING_LEN   sizeof(struct TPDO_MAPPING)

#endif

#ifdef PANOSONIC_DRIVER

typedef struct RPDO_MAPPING
{
	uint16_t CtrlWord;
	uint8_t Modes;
	int TargetPos;
	//uint32_t 	Do;
	//uint32_t	DoEnable;
}RPDO_MAPPING,*PRPDO_MAPPING;
#define RPDO_MAPPING_LEN   sizeof(struct RPDO_MAPPING)

typedef struct TPDO_MAPPING
{
	uint16_t	ErroCode;
	uint16_t StatusWord;
	uint8_t ModesDisplay;
	int ActualPosition;
	uint16_t	TouchProbeStatus;
	int	TouchProbePosValue;
	int 	FollwErroValue;
	short ActualTorq;
}TPDO_MAPPING,*PTRPDO_MAPPING;
#define TPDO_MAPPING_LEN   sizeof(struct TPDO_MAPPING)

typedef struct CST_RPDO_MAPPING
{
	uint16_t CtrlWord;
	uint8_t Modes;
	short TargetTorq;
	//uint32_t 	Do;
	//uint32_t	DoEnable;
}CST_RPDO_MAPPING,*PCST_RPDO_MAPPING;
#define CST_RPDO_MAPPING_LEN   sizeof(struct CST_RPDO_MAPPING)

#endif

#ifdef COOL_DRIVER
typedef struct RPDO_MAPPING
{
/*	uint16_t CtrlWord;
	int TargetPos;
	int TargetVel;
	int TargetTq;
	uint8_t Modes;
	uint8_t res;*/
	uint16_t CtrlWord;
	int TargetPos;
	int PosOffset;
	int VelOffset;
	short TqOffset;
	uint8_t Modes;
	uint8_t res;
}RPDO_MAPPING,*PRPDO_MAPPING;
#define RPDO_MAPPING_LEN   sizeof(struct RPDO_MAPPING)

typedef struct TPDO_MAPPING
{

/*	uint16_t StatusWord;
	int ActualPosition;
	int ActualVel;
	short ActualTq;
	uint8_t ModesDisplay;
	uint8_t res;
	int ActualPositionErr;*/
	uint16_t StatusWord;
	int ActualPosition;
	//int ActualVel;
	short ActualTq;
	uint8_t ModesDisplay;
	uint8_t res;
}TPDO_MAPPING,*PTRPDO_MAPPING;
#define TPDO_MAPPING_LEN   sizeof(struct TPDO_MAPPING)
#endif

#ifdef ZZ_DRIVER
typedef struct RPDO_MAPPING
{
/*	uint16_t CtrlWord;
	int TargetPos;
	int TargetVel;
	int TargetTq;
	uint8_t Modes;
	uint8_t res;*/
	uint16_t CtrlWord;
	int TargetPos;
	int PosOffset;
	int VelOffset;
	short TqOffset;
	uint8_t Modes;
	uint8_t res;
}RPDO_MAPPING,*PRPDO_MAPPING;
#define RPDO_MAPPING_LEN   sizeof(struct RPDO_MAPPING)

typedef struct TPDO_MAPPING
{

/*	uint16_t StatusWord;
	int ActualPosition;
	int ActualVel;
	short ActualTq;
	uint8_t ModesDisplay;
	uint8_t res;
	int ActualPositionErr;*/
	uint16_t StatusWord;
	int ActualPosition;
	int ActualVel;
	short ActualTq;
	uint8_t ModesDisplay;
	uint8_t res;
}TPDO_MAPPING,*PTRPDO_MAPPING;
#define TPDO_MAPPING_LEN   sizeof(struct TPDO_MAPPING)
#endif

typedef struct CSV_RPDO_MAPPING
{
	uint16_t CtrlWord;
	uint16_t LenzeCtrl;
	uint8_t Modes;
	short int TorqueOffset;
	int TargetVelocity;
	uint16_t PosTorqueLimit;
	uint16_t NegTorqueLimit;
}CSV_RPDO_MAPPING,*PCSV_RPDO_MAPPING;
#define CSV_RPDO_MAPPING_LEN   sizeof(struct CSV_RPDO_MAPPING)

typedef struct COE_WRITE
{
	uint16_t index;
	uint8_t subindex;
	uint8_t len;
	uint32_t data;
}COE_WRITE,*PCOE_WRITE;
#define COE_WRITE_LEN   sizeof(struct COE_WRITE)

typedef struct IO_RPDO_MAPPING
{
	uint32_t Output0;
	uint32_t Output1;
}IO_RPDO_MAPPING,*PIO_RPDO_MAPPING;
#define IO_RPDO_MAPPING_LEN   sizeof(struct IO_RPDO_MAPPING)

typedef struct IO_TPDO_MAPPING
{
	uint32_t Input0;
	uint32_t Input1;
}IO_TPDO_MAPPING,*PIO_TRPDO_MAPPING;
#define IO_TPDO_MAPPING_LEN   sizeof(struct IO_TPDO_MAPPING)


enum CIA402_MODES{VL=2,CSP=8,CSV=9,CST=10};////伺服控制模式
enum DEVICE_TYPE{SERVO=1,IO=2};////从站设备类型

void CallBack(u_char * arg, const struct pcap_pkthdr * pkthdr, const  u_char * packet);


#endif /* ETHSERVICE_H_ */
