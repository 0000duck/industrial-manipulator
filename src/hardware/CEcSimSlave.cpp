/*
 * CEcSimSlave.cpp
 *
 *  Created on: 2015年12月16日
 *      Author: root
 */


#include "CEcSimSlave.h"

CEcSimSlave::CEcSimSlave(DEVICE_TYPE type) {
	// TODO Auto-generated constructor stub
	dv_type=type;
	 m_cStatus=0;
	 m_nStatusCode=0;
	m_nDlAddr=0;
	m_nOutOffset=0;
	m_nOutLen=0;
	m_nInOffset=0;
	m_nInLen=0;
	int i;
	for(i=0;i<4;i++)
	{
		m_pSyncM[i].m_cStatus=0;
		m_pSyncM[i].m_ncActive=0x01;
		m_pSyncM[i].m_cPdiContrl=0;
	}
	if(type==IO){
		m_pSyncM[0].m_nPhyStart=0x1000;
		m_pSyncM[0].m_nLength=0x0100;
		m_pSyncM[0].m_cContrl=0x26;

		m_pSyncM[1].m_nPhyStart=0x1400;
		m_pSyncM[1].m_nLength=0x0100;
		m_pSyncM[1].m_cContrl=0x22;

		m_pSyncM[2].m_nPhyStart=0x1800;
		m_pSyncM[2].m_nLength=IO_RPDO_MAPPING_LEN;
		m_pSyncM[2].m_cContrl=0x64;

		m_pSyncM[3].m_nPhyStart=0x1c00;
		m_pSyncM[3].m_nLength=IO_TPDO_MAPPING_LEN;
		m_pSyncM[3].m_cContrl=0x20;
	}
	else{ //servo
		m_pSyncM[0].m_nPhyStart=0x1000;
		m_pSyncM[0].m_nLength=0x0100;
		m_pSyncM[0].m_cContrl=0x26;

		m_pSyncM[1].m_nPhyStart=0x1400;
		m_pSyncM[1].m_nLength=0x0100;
		m_pSyncM[1].m_cContrl=0x22;

		m_pSyncM[2].m_nPhyStart=0x1800;
		//m_pSyncM[2].m_nLength=12;
		m_pSyncM[2].m_nLength=RPDO_MAPPING_LEN;
		m_pSyncM[2].m_cContrl=0x64;

		m_pSyncM[3].m_nPhyStart=0x1c00;
		//m_pSyncM[3].m_nLength=14;
		m_pSyncM[3].m_nLength=TPDO_MAPPING_LEN;
		m_pSyncM[3].m_cContrl=0x20;
#ifdef I700_DRIVER
		m_pSyncM[0].m_nLength=0x0200;
		m_pSyncM[1].m_nLength=0x0200;
		m_pSyncM[1].m_nPhyStart=0x1200;
		m_pSyncM[2].m_nPhyStart=0x1400;
		m_pSyncM[3].m_nPhyStart=0x1700;
#endif
#ifdef ZZ_DRIVER
	m_pSyncM[0].m_nLength=0x0c0;
	m_pSyncM[1].m_nLength=0x0c0;
	m_pSyncM[2].m_cContrl=0x24;
#endif
	}


}

CEcSimSlave::~CEcSimSlave() {
	// TODO Auto-generated destructor stub
}

void CEcSimSlave::SetSM(int i,uint16_t PhyStart,uint16_t Length,uint8_t Contrl){
	m_pSyncM[i].m_nPhyStart=PhyStart;
	m_pSyncM[i].m_nLength=Length;
	m_pSyncM[i].m_cContrl=Contrl;
}


