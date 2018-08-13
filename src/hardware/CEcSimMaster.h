/*
 * CEcSimMaster.h
 *
 *  Created on: 2015年12月16日
 *      Author: root
 */

#include "CEcPcapDevice.h"
#include "CEcNetDevice.h"
#include "CEcSimSlave.h"

#ifndef CECSIMMASTER_H_
#define CECSIMMASTER_H_

class CEcSimMaster {
public:
	/**
	 * @brief
	 * @param slavnum [in] slave个数
	 * @param type [in] slave的种类, 1:servo; 2:IO
	 */
	CEcSimMaster(int slavnum,DEVICE_TYPE type[]);
	virtual ~CEcSimMaster();
	void CreatSlave(int i);
	/**
	 * @brief 打开CEcNetDevice和CEcPcapDevice
	 * @return 若打开失败, 返回false
	 */
	bool open();
	void WriteSM(int i);
	void WriteFMMU(int i);
	int ReadALStatus();
	int ReadALStatusCode();
	void ReadALStatus(uint8_t lenth);
	void WriteALContrl(uint16_t state);
	void SetSlvAddr();
	void PreStateMachine();
	int StateMachine();
	void Release();
	void ImageAssign();
	void SendMail(
			uint8_t ecatindex,
			uint8_t slvcnt,
			uint16_t index,
			uint8_t subindex,
			uint8_t len,
			uint32_t maildata);
	void SendMail(
			uint8_t ecatindex,
			uint8_t slvcnt,
			uint8_t sdocmd,
			uint16_t index,
			uint8_t subindex,
			uint32_t len,
			uint64_t lmaildata);
	void ReadMail(uint8_t slvcnt,uint8_t index);
	void SendECAT(
                uint8_t  cmd,
	            uint8_t  idx,
            	uint16_t slvCnt,
            	uint16_t ofsAdr,
            	uint16_t length,
				uint8_t * pECATData);
	void SSendECAT(
                uint8_t  cmd,
	            uint8_t  idx,
            	uint16_t slvNum,
            	uint16_t ofsAdr,
            	uint16_t length,
				uint8_t * pECATData);
	void SendFMMU();
	void InitConfig();
	void WatchDog();
	void CheckSM(uint8_t slvNum);
	void ReMail(uint8_t slvNum);
	void SycnDC();
	void SycnInit();
	void SendCyclicFrame(int cnt,uint8_t ECATIndex,uint8_t *pRpdoData);
	void SendCyclicFrame(int cnt,uint8_t ECATIndex,RPDO_MAPPING *pRpdoData);
	void ServoParamInit();
	void PdoAssign(uint8_t slvnum,uint16_t index,uint16_t obj_assign[],uint8_t num);
	void PdoMapping(uint8_t slvnum,uint16_t index,uint32_t obj_mapping[],uint8_t num);
	void ReadServoParam();
	void DeviceControl();
	void RealTimeSetting();
	void ReadESC();
	void Read220();
	void AL_ErroHandl(int alstatus);
/*	void Delay(uint32_t delaytime_us);*/
	void CoEWrite(uint8_t slvnum,COE_WRITE *writedata);
	uint32_t	CoERead(uint8_t slvnum,COE_WRITE *writedata);
	//void ErroIndAck();

public:
	int m_nStatus;
	int m_nRequie;
	int m_nEcSlave;
	uint16_t m_nReadStatus;
	uint8_t m_OutputImage[ETHERNET_MAX_FRAME_LEN] ;
	CEcSimSlave   ** m_ppEcSlave;
	CEcNetDevice   * m_pNetdev;
	CEcPcapDevice    * m_npPcapdev;

	unsigned char * pcap_packet;
	int packetlen;
	int mailcnt;
	int LgDataLenth[2];

};

#endif /* CECSIMMASTER_H_ */
