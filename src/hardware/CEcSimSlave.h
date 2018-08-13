/*
 * CEcSimSlave.h
 *
 *  Created on: 2015年12月16日
 *      Author: root
 */
#include "EthService.h"

#ifndef CECSIMSLAVE_H_
#define CECSIMSLAVE_H_

class CEcSimSlave {
public:
	CEcSimSlave(DEVICE_TYPE type);
	virtual ~CEcSimSlave();
	void SetSM(int i,uint16_t PhyStart,uint16_t Length,uint8_t Contrl);

public:
	/**> FMMU */
	FMMU    m_pFmmu[4];

	/**> SyncManager */
	SYNCMANAGE m_pSyncM[4];

	/**> 状态字 */
	uint16_t     m_cStatus;

	/**> 状态码 */
	uint16_t     m_nStatusCode;

	/**> 数据链路层地址 */
	uint16_t     m_nDlAddr;

	/**> 从站输出数据存储偏移地址 */
	uint16_t     m_nOutOffset;

	/**> 从站输出数据长度 */
	uint16_t     m_nOutLen;

	/**> 从站输入数据存储偏移地址 */
	uint16_t     m_nInOffset;

	/**> 从站输入数据长度 */
	uint16_t     m_nInLen;

	/**> 从站类型 */
	DEVICE_TYPE dv_type;
};

#endif /* CECSIMSLAVE_H_ */
