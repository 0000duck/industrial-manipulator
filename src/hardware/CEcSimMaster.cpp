/*
 * CEcSimMaster.cpp
 *
 *  Created on: 2015年12月16日
 *      Author: root
 */

#include "CEcSimMaster.h"

CEcSimMaster::CEcSimMaster(int slavnum,DEVICE_TYPE type[]) {
	// TODO Auto-generated constructor stub
	m_nEcSlave=slavnum;
	m_nStatus=10;
	m_nRequie=0;
	m_nReadStatus=0;
	m_ppEcSlave=new CEcSimSlave*[m_nEcSlave];
	memset(m_ppEcSlave,0,m_nEcSlave*sizeof(CEcSimSlave*));
	for(int i=0;i<m_nEcSlave;i++)
	{
		m_ppEcSlave[i]=new CEcSimSlave(type[i]);
		m_ppEcSlave[i]->m_nDlAddr=1001+i;
	}
	m_pNetdev=new CEcNetDevice;
	m_npPcapdev=new CEcPcapDevice;
	pcap_packet=(unsigned char*)malloc(ETHERNET_MAX_FRAME_LEN);
	packetlen=0;
	mailcnt=1;
}

CEcSimMaster::~CEcSimMaster() {
	// TODO Auto-generated destructor stub
	//m_npPcapdev->Closedev();
	//m_pNetdev->Closedev();
	delete m_pNetdev;
	delete m_npPcapdev;
	for(int i=0;i<m_nEcSlave;i++)
	{
		delete m_ppEcSlave[i];
	}
	free(pcap_packet);
}

/*void CEcSimMaster::CreatSlave(int i)
{
	m_ppEcSlave[i]=new CEcSimSlave;
	m_ppEcSlave[i]->m_nDlAddr=1001+i;
}*/

bool CEcSimMaster::open(){
	int erro;
	erro=m_pNetdev->Opendev();
	if(erro)
		return false;
	erro=m_npPcapdev->Opendev();
	if(erro)
		return false;
	return 1;
}

void CEcSimMaster::SendECAT(
            uint8_t  cmd,
            uint8_t  idx,
        	uint16_t slvCnt,
        	uint16_t ofsAdr,
        	uint16_t length,
			uint8_t * pECATData){

	        int framelen=0;
			TETHERNET_88A4_MAX_FRAME ethFrame;
			ethFrame.E88A4.Reserved=0;
			ethFrame.E88A4.Type=1;

			ECAT_MAX_FRAME ecHdr;
			ecHdr.ECATHeader.cmd=cmd;
			ecHdr.ECATHeader.idx=idx;
			ecHdr.ECATHeader.ofsAdr=ofsAdr;
			ecHdr.ECATHeader.length=length;
			ecHdr.ECATHeader.next=1;
			ecHdr.ECATHeader.res=0;
			ecHdr.ECATHeader.irq=0;

			uint8_t * pData=ethFrame.Data;
			for(int i=0;i<slvCnt;i++)
			{
				    ecHdr.ECATHeader.slAdr=1001+i;
					memcpy(pData,&ecHdr,10);
					pData=pData+10;
					framelen=framelen+10;
					////////////////////////////////////////////////////	ECATData
					memcpy(pData,pECATData+i*length,length);
					pData=pData+length;
					framelen=framelen+length;
					////////////////////////////////////////////////////WKC
					pData[0]=0;
					pData[1]=0;
					pData=pData+2;
					framelen=framelen+2;
			}

			pData=pData-(length+5);
			pData[0]&=0x7F;

			ethFrame.E88A4.Length=framelen;
			int erro=0;
			while(erro<1)//若接收失败则重新发送
			{
				erro=m_pNetdev->SendPacket(framelen+2,(unsigned char *)&ethFrame);
				erro=m_npPcapdev->Receiver();
			}
}
void CEcSimMaster::SSendECAT(
            uint8_t  cmd,
            uint8_t  idx,
        	uint16_t slvNum,
        	uint16_t ofsAdr,
        	uint16_t length,
			uint8_t * pECATData){

	        int framelen=0;
			TETHERNET_88A4_MAX_FRAME ethFrame;
			ethFrame.E88A4.Reserved=0;
			ethFrame.E88A4.Type=1;

			ECAT_MAX_FRAME ecHdr;
			ecHdr.ECATHeader.cmd=cmd;
			ecHdr.ECATHeader.idx=idx;
			ecHdr.ECATHeader.ofsAdr=ofsAdr;
			ecHdr.ECATHeader.slAdr=m_ppEcSlave[slvNum]->m_nDlAddr;
			ecHdr.ECATHeader.length=length;
			ecHdr.ECATHeader.next=0;
			ecHdr.ECATHeader.res=0;
			ecHdr.ECATHeader.irq=0;

			uint8_t * pData=ethFrame.Data;
			memcpy(pData,&ecHdr,10);
			pData=pData+10;
			framelen=framelen+10;
			////////////////////////////////////////////////////	ECATData
			memcpy(pData,pECATData,length);
			pData=pData+length;
			framelen=framelen+length;
			////////////////////////////////////////////////////WKC
			pData[0]=0;
			pData[1]=0;
			pData=pData+2;
			framelen=framelen+2;

			pData=pData-(length+5);
			pData[0]&=0x7F;

			ethFrame.E88A4.Length=framelen;
			int erro=0;
			while(erro<1)//若接收失败则重新发送
			{
				erro=m_pNetdev->SendPacket(framelen+2,(unsigned char *)&ethFrame);
				erro=m_npPcapdev->Receiver();
			}
}
void CEcSimMaster::SendFMMU(){

	        int framelen=0;
			TETHERNET_88A4_MAX_FRAME ethFrame;
			ethFrame.E88A4.Reserved=0;
			ethFrame.E88A4.Type=1;

			ECAT_MAX_FRAME ecHdr;
			ecHdr.ECATHeader.cmd=12;
			ecHdr.ECATHeader.idx=0x80;
			ecHdr.ECATHeader.lgAdr=0x00010800;
			ecHdr.ECATHeader.length=135;
			ecHdr.ECATHeader.next=0;
			ecHdr.ECATHeader.res=0;
			ecHdr.ECATHeader.irq=0;

			uint8_t * pData=ethFrame.Data;
			memcpy(pData,&ecHdr,10);
			pData=pData+10;
			framelen=framelen+10;
			////////////////////////////////////////////////////Data
			for(int i=0;i<ecHdr.ECATHeader.length;i++)
			{
				pData[i]=0;
			}
			pData=pData+ecHdr.ECATHeader.length;
			framelen=framelen+ecHdr.ECATHeader.length;
			////////////////////////////////////////////////////WKC
			pData[0]=0;
			pData[1]=0;
			pData=pData+2;
			framelen=framelen+2;

			ethFrame.E88A4.Length=framelen;
			int erro=0;
			while(erro<1)//若接收失败则重新发送
			{
				erro=m_pNetdev->SendPacket(framelen+2,(unsigned char *)&ethFrame);
				erro=m_npPcapdev->Receiver();
			}
}

void CEcSimMaster::WriteSM(int i){
	int slvCnt;

	uint8_t  cmd=5;
	uint8_t  idx=0x82;
	uint16_t ofsAdr=0x0800+i*8;
	uint16_t length=8;

	uint8_t * pData=(uint8_t*)malloc(length*m_nEcSlave);
	uint8_t *p=pData;
	for(slvCnt=0;slvCnt<m_nEcSlave;slvCnt++)
	{
		memcpy(p,&(m_ppEcSlave[slvCnt]->m_pSyncM[i]),length);
		p=p+length;
	}
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);

	free(pData);
}
void CEcSimMaster::WriteFMMU(int i){
	int slvCnt;

	uint8_t  cmd=5;
	uint8_t  idx=0x82;
	uint16_t ofsAdr=0x0600+i*16;
	uint16_t length=16;

	uint8_t * pData=(uint8_t*)malloc(length*m_nEcSlave);
	uint8_t *p=pData;
	for(slvCnt=0;slvCnt<m_nEcSlave;slvCnt++)
	{
		memcpy(p,&(m_ppEcSlave[slvCnt]->m_pFmmu[i]),length);
		p=p+length;
	}
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);

}
int CEcSimMaster::ReadALStatus(){
		uint16_t *pstatu;
		uint16_t status;
		uint8_t * pData=(uint8_t*)malloc(2);
		uint16_t ECATData=0;
		int frmoffset;
		memcpy(pData,&ECATData,2);
		SendECAT(7,0x82,1,0x130,2,pData);

		TETYPE_ECAT_HEADER *pECATframe;
		frmoffset=TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN;
	    pECATframe=(TETYPE_ECAT_HEADER*)(m_npPcapdev->pcap_packet+frmoffset);
	    if(pECATframe->ofsAdr==0x130){
		    frmoffset=TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN+TETYPE_ECAT_HEADER_LEN;
			pstatu=(uint16_t *)(m_npPcapdev->pcap_packet+frmoffset);
			status=*pstatu;
			free(pData);
			return status;
	    }
	    else{
	    	free(pData);
	    	return -1;
	    }
}
int CEcSimMaster::ReadALStatusCode(){
	uint16_t *pstatu_code;
	uint8_t * pData=(uint8_t*)malloc(2);
	uint16_t ECATData=0;
	int frmoffset;
	memcpy(pData,&ECATData,2);
	SendECAT(7,0x82,1,0x134,2,pData);

	TETYPE_ECAT_HEADER *pECATframe;
	frmoffset=TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN;
    pECATframe=(TETYPE_ECAT_HEADER*)(m_npPcapdev->pcap_packet+frmoffset);
    if(pECATframe->ofsAdr==0x134){
	    frmoffset=TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN+TETYPE_ECAT_HEADER_LEN;
	    pstatu_code=(uint16_t *)(m_npPcapdev->pcap_packet+frmoffset);
		free(pData);
		return *pstatu_code;
    }
    else{
    	free(pData);
    	return -1;
    }
}
void CEcSimMaster::ReadALStatus(uint8_t lenth){
		uint16_t *pstatu;
		uint8_t * pData=(uint8_t*)malloc(lenth*m_nEcSlave);
		uint8_t ECATData[lenth*m_nEcSlave];
		int frmoffset=TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN+TETYPE_ECAT_HEADER_LEN;

		for(int i=0;i<lenth*m_nEcSlave;i++)
		{
			ECATData[i]=0;
		}
		memcpy(pData,&ECATData,lenth*m_nEcSlave);
		SendECAT(4,0x82,m_nEcSlave,0x130,lenth,pData);
		pstatu=(uint16_t *)(m_npPcapdev->pcap_packet+frmoffset);
		cout<<"ETHERCAT STATUS:"<<*pstatu<<",  ";
		if(lenth==6){
			uint16_t *AL_code;
			AL_code=(uint16_t *)(m_npPcapdev->pcap_packet+frmoffset+4);
			cout<<"ETHERCAT AL CODE:"<<*AL_code<<"\n";
		}
		free(pData);
}
void CEcSimMaster::WriteALContrl(uint16_t state){
	int slvCnt;

	uint8_t  cmd=5;
	uint8_t  idx=0x82;
	uint16_t ofsAdr=0x120;
	uint16_t length=2;

	uint8_t * pData=(uint8_t*)malloc(length*m_nEcSlave);
	uint8_t *p=pData;
	for(slvCnt=0;slvCnt<m_nEcSlave;slvCnt++)
	{
		memcpy(p,&state,length);
		if(slvCnt<m_nEcSlave-1)
			p=p+length;
	}
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);
	free(pData);
}
void CEcSimMaster::ReadESC(){
	int framelen=0;
	int slvCnt;

	uint8_t ReadData[20];
	for(int i=0;i<20;i++)
	{
		ReadData[i]=0;
	}

	TETHERNET_88A4_MAX_FRAME ethFrame;
	ethFrame.E88A4.Reserved=0;
	ethFrame.E88A4.Type=1;
	ECAT_MAX_FRAME ecHdr;

	ecHdr.ECATHeader.cmd=1;
	ecHdr.ECATHeader.idx=0x82;
	ecHdr.ECATHeader.slAdr=0x0000;
	ecHdr.ECATHeader.ofsAdr=0x0000;
	ecHdr.ECATHeader.length=20;
	ecHdr.ECATHeader.next=1;
	ecHdr.ECATHeader.res=0;
	ecHdr.ECATHeader.irq=0;

	uint8_t * pData=ethFrame.Data;
	memcpy(pData,&ecHdr,10);
	pData=pData+10;
	framelen=framelen+10;
	memcpy(pData,&ReadData,ecHdr.ECATHeader.length);
	pData=pData+ecHdr.ECATHeader.length;
	framelen=framelen+ecHdr.ECATHeader.length;
	pData[0]=0;
	pData[1]=0;
	pData=pData+2;
	framelen=framelen+2;

	//////////////////////////////////////////////////////第2个从站
	if(m_nEcSlave>1)
	{
		for(slvCnt=1;slvCnt<m_nEcSlave;slvCnt++)
		{
			ecHdr.ECATHeader.slAdr=0x0000-slvCnt;
			//ecHdr.ECATHeader.next=0;
			memcpy(pData,&ecHdr,10);
			pData=pData+10;
			framelen=framelen+10;
			memcpy(pData,&ReadData,ecHdr.ECATHeader.length);
			pData=pData+ecHdr.ECATHeader.length;
			framelen=framelen+ecHdr.ECATHeader.length;
			pData[0]=0;
			pData[1]=0;
			pData=pData+2;
			framelen=framelen+2;
		}

	}

	pData=pData-25;
	pData[0]&=0x7F;

	ethFrame.E88A4.Length=framelen;
	int erro=0;
	while(erro<1)//若接收失败则重新发送
	{
		erro=m_pNetdev->SendPacket(framelen+2,(unsigned char *)&ethFrame);
		erro=m_npPcapdev->Receiver();
	}
}
void CEcSimMaster::SetSlvAddr(){
	int framelen=0;
	int slvCnt;

	TETHERNET_88A4_MAX_FRAME ethFrame;
	ethFrame.E88A4.Reserved=0;
	ethFrame.E88A4.Type=1;
	ECAT_MAX_FRAME ecHdr;

	//////////////////////////////////////////////////////第1个从站
	ecHdr.ECATHeader.cmd=2;
	ecHdr.ECATHeader.idx=0x82;
	ecHdr.ECATHeader.slAdr=0x0000;
	ecHdr.ECATHeader.ofsAdr=0x0010;
	ecHdr.ECATHeader.length=2;
	ecHdr.ECATHeader.next=1;
	ecHdr.ECATHeader.res=0;
	ecHdr.ECATHeader.irq=0;

	uint8_t * pData=ethFrame.Data;
	memcpy(pData,&ecHdr,10);
	pData=pData+10;
	framelen=framelen+10;
	memcpy(pData,&(m_ppEcSlave[0]->m_nDlAddr),2);
	pData=pData+2;
	framelen=framelen+2;
	pData[0]=0;
	pData[1]=0;
	pData=pData+2;
	framelen=framelen+2;

	//////////////////////////////////////////////////////第2个从站
	if(m_nEcSlave>1)
	{
		for(slvCnt=1;slvCnt<m_nEcSlave;slvCnt++)
		{
			ecHdr.ECATHeader.slAdr=0x0000-slvCnt;
			//ecHdr.ECATHeader.next=0;
			memcpy(pData,&ecHdr,10);
			pData=pData+10;
			framelen=framelen+10;
			memcpy(pData,&(m_ppEcSlave[slvCnt]->m_nDlAddr),2);
			pData=pData+2;
			framelen=framelen+2;
			pData[0]=0;
			pData[1]=0;
			pData=pData+2;
			framelen=framelen+2;
		}
	}

	pData=pData-7;
	pData[0]&=0x7F;

	ethFrame.E88A4.Length=framelen;
	int erro=0;
	while(erro<1)//若接收失败则重新发送
	{
		erro=m_pNetdev->SendPacket(framelen+2,(unsigned char *)&ethFrame);
		erro=m_npPcapdev->Receiver();
	}
}
void CEcSimMaster::PreStateMachine(){
	int alstatus;
	m_nStatus=10;
	do{
		alstatus=ReadALStatus();
		if((alstatus&0x01)==1)///////////////////////判断当前状态，应取各从站最低者
			m_nStatus=10;
		else if((alstatus&0x02)==2)
			m_nStatus=20;
		else if((alstatus&0x04)==4)
			m_nStatus=40;
		else if((alstatus&0x08)==8)
			m_nStatus=80;
	}while(alstatus<0);
	cout<<"PRE ETHERCAT STATUS:"<<alstatus<<"\n";
}
int CEcSimMaster::StateMachine(){
	int alstatus;
	int log_len=0;
	for(int i=0;i<m_nEcSlave;i++){
		log_len+=m_ppEcSlave[i]->m_pSyncM[2].m_nLength;
	}
	uint8_t *pdo_data=(uint8_t*)malloc(log_len);
	memset(pdo_data,0,log_len);
	uint8_t EcatIndex=0;
	uint8_t* pIndex;
	int FrmDelay=1;
	TETYPE_ECAT_HEADER *pECATframe;
	uint16_t* pstatu;
	int err;

	while(m_nRequie>m_nStatus){
		switch(m_nStatus){
		case 10:
			SetSlvAddr();
			usleep(PDO_CYCL_TIME);
			//ReadESC();
/*			WriteALContrl(1);
			cout<<"REQUIRE FOR ETHERCAT STATUS: 1"<<"\n";
			do{
				alstatus=ReadALStatus();
			}while(alstatus<0);
			cout<<"ETHERCAT STATUS:"<<alstatus<<"\n";*/
			InitConfig();
			WriteSM(0);
			usleep(PDO_CYCL_TIME);
			WriteSM(1);
			usleep(PDO_CYCL_TIME);
			//ImageAssign();
			WriteFMMU(2);
			usleep(PDO_CYCL_TIME);
			WatchDog();
			WriteALContrl(2);
			cout<<"REQUIRE FOR ETHERCAT STATUS: 2"<<"\n";
			m_nStatus=11;
			break;
		case 11:
			do{
				alstatus=ReadALStatus();
			}while(alstatus<0);
			cout<<"ETHERCAT STATUS:"<<alstatus<<"\n";
			m_nStatus=12;
			break;
		case 12:
			if((alstatus&0x001f)==2){////////////判断是否到达请求状态，需全部从站一致
				m_nStatus=20;
			}
			else if((alstatus&0x0010)>0){
				m_nStatus=13;
				AL_ErroHandl(alstatus);
			}
			else{
				m_nStatus=11;
			}
			break;
		case 20:
			WriteSM(2);
			usleep(PDO_CYCL_TIME);
			WriteSM(3);
			usleep(PDO_CYCL_TIME);
			WriteFMMU(0);
			usleep(PDO_CYCL_TIME);
			WriteFMMU(1);
			usleep(PDO_CYCL_TIME);
			WriteALContrl(4);
			cout<<"REQUIRE FOR ETHERCAT STATUS: 4"<<"\n";
			m_nStatus=21;
			break;
		case 21:///////////////////////////////////////////////read
/*			pIndex=(uint8_t *)(m_npPcapdev->pcap_packet+TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN+1);
			if(EcatIndex<*pIndex)
			{
				FrmDelay=EcatIndex+0x80-*pIndex;
			}
			else{
				FrmDelay=EcatIndex-*pIndex;
			}*/
			for(int i=0;i<20;i++){
				usleep(PDO_CYCL_TIME);
				SendCyclicFrame(FrmDelay,EcatIndex,pdo_data);
				err=m_npPcapdev->CyclReceiver(1);
				EcatIndex++;
				if(EcatIndex>=0x80){
					EcatIndex=0;
				}
			}

		    pECATframe=(TETYPE_ECAT_HEADER*)(m_npPcapdev->pcap_packet+TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN);
		    if(pECATframe->ofsAdr==0x130){
				pstatu=(uint16_t *)(m_npPcapdev->pcap_packet+TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN+TETYPE_ECAT_HEADER_LEN);
				alstatus=*pstatu;
		    }
			cout<<"ETHERCAT STATUS:"<<alstatus<<"\n";
			m_nStatus=22;
			break;
		case 22:
			if((alstatus&0x001f)==4){
				m_nStatus=40;
			}
			else if((alstatus&0x0010)>0){
				m_nStatus=23;
				return alstatus;/////////////////////////////pre-s状态切换不成功，需重新进行PDO配置
				//AL_ErroHandl(alstatus);
			}
			else{
				m_nStatus=21;
			}
			break;
		case 40:////////////////////////////////require
			WriteALContrl(8);
			cout<<"REQUIRE FOR ETHERCAT STATUS: 8"<<"\n";
			m_nStatus=41;
			break;
		case 41://///////////////////////read
/*			pIndex=(uint8_t *)(m_npPcapdev->pcap_packet+TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN+1);
			if(EcatIndex<*pIndex)
			{
				FrmDelay=EcatIndex+0x80-*pIndex;
			}
			else{
				FrmDelay=EcatIndex-*pIndex;
			}*/
			for(int i=0;i<20;i++){
				usleep(PDO_CYCL_TIME);
				SendCyclicFrame(FrmDelay,EcatIndex,pdo_data);
				err=m_npPcapdev->CyclReceiver(1);
				EcatIndex++;
				if(EcatIndex>=0x80){
					EcatIndex=0;
				}
			}

		    pECATframe=(TETYPE_ECAT_HEADER*)(m_npPcapdev->pcap_packet+TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN);
		    if(pECATframe->ofsAdr==0x130){
				pstatu=(uint16_t *)(m_npPcapdev->pcap_packet+TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN+TETYPE_ECAT_HEADER_LEN);
				alstatus=*pstatu;
		    }
			cout<<"ETHERCAT STATUS:"<<alstatus<<"\n";
			m_nStatus=42;
			break;
		case 42://////////////////////////////////check
			if((alstatus&0x001f)==8){
				m_nStatus=80;
			}
			else if((alstatus&0x0010)>0){
				if((alstatus & 0x02)>0){/////////////////////////////判断当前状态
					m_nStatus=23;
					return alstatus;/////////////////////////////pre-s状态切换不成功，需重新进行PDO配置
				}
				else{
					m_nStatus=43;
					AL_ErroHandl(alstatus);
				}
			}
			else{
				m_nStatus=41;
			}
			break;
		}
	}
	if(m_nRequie<m_nStatus){
		WriteALContrl(m_nRequie/10);
		alstatus=ReadALStatus();
		m_nStatus=alstatus;
	}
	return alstatus;
}

void CEcSimMaster::AL_ErroHandl(int alstatus){
	int statuscode;
	do{
		statuscode=ReadALStatusCode();
	}while(statuscode<0);
	cout<<"ETHERCAT STATUS CODE:"<<statuscode<<"\n";
	switch(m_nStatus){
	case 13:
		WriteALContrl(0x11);///////////////////////////错误复位，状态退回
		m_nStatus=10;/////////////////////////////////重新进行配置，重新申请状态请求
		usleep(4000);
		break;
	case 23:
		WriteALContrl(0x12);
		m_nStatus=20;
		usleep(4000);
		break;
	case 43:
		WriteALContrl(0x14);
		m_nStatus=40;
		usleep(4000);
		break;
	case 83:
		WriteALContrl(0x18);
		usleep(4000);
		break;
	}
	ReadALStatus();
	//exit(0);
}

void CEcSimMaster::Release(){
	m_npPcapdev->Closedev();
	m_pNetdev->Closedev();
}
void CEcSimMaster::ImageAssign(){
	int slvCnt;
	uint16_t AddrOffset[2]={0,0};
	for(slvCnt=0;slvCnt<m_nEcSlave;slvCnt++)
	{
		if(slvCnt>0){
			AddrOffset[0]=AddrOffset[0]+m_ppEcSlave[slvCnt-1]->m_pSyncM[2].m_nLength;
			AddrOffset[1]=AddrOffset[1]+m_ppEcSlave[slvCnt-1]->m_pSyncM[3].m_nLength;
		}

		m_ppEcSlave[slvCnt]->m_nOutLen=0x0001;
		m_ppEcSlave[slvCnt]->m_nInLen=0x0001;
		/////////////////////////////////////////////从站0,输出FMMU
		m_ppEcSlave[slvCnt]->m_pFmmu[0].m_nLgstartAdr=0x00010000+AddrOffset[0];
		m_ppEcSlave[slvCnt]->m_pFmmu[0].m_nLength=m_ppEcSlave[slvCnt]->m_pSyncM[2].m_nLength;
		m_ppEcSlave[slvCnt]->m_pFmmu[0].m_nLgStartBit=0;
		m_ppEcSlave[slvCnt]->m_pFmmu[0].m_nLgStopBit=0x07;
		m_ppEcSlave[slvCnt]->m_pFmmu[0].m_nPhyStartAdr=m_ppEcSlave[slvCnt]->m_pSyncM[2].m_nPhyStart;
		m_ppEcSlave[slvCnt]->m_pFmmu[0].m_nPhyStartBit=0;
		m_ppEcSlave[slvCnt]->m_pFmmu[0].m_nType=0x02;
		m_ppEcSlave[slvCnt]->m_pFmmu[0].m_nActive=0x01;

		//////////////////////////////////////////////从站0,输入FMMU
		m_ppEcSlave[slvCnt]->m_pFmmu[1].m_nLgstartAdr=0x00010800+AddrOffset[1];
		m_ppEcSlave[slvCnt]->m_pFmmu[1].m_nLength=m_ppEcSlave[slvCnt]->m_pSyncM[3].m_nLength;
		m_ppEcSlave[slvCnt]->m_pFmmu[1].m_nLgStartBit=0;
		m_ppEcSlave[slvCnt]->m_pFmmu[1].m_nLgStopBit=0x07;
		m_ppEcSlave[slvCnt]->m_pFmmu[1].m_nPhyStartAdr=m_ppEcSlave[slvCnt]->m_pSyncM[3].m_nPhyStart;
		m_ppEcSlave[slvCnt]->m_pFmmu[1].m_nPhyStartBit=0;
		m_ppEcSlave[slvCnt]->m_pFmmu[1].m_nType=0x01;
		m_ppEcSlave[slvCnt]->m_pFmmu[1].m_nActive=0x01;

		//////////////////////////////////////////////从站0输入FMMU'
		m_ppEcSlave[slvCnt]->m_pFmmu[2].m_nLgstartAdr=0x00080000;//+slvCnt*8;
		m_ppEcSlave[slvCnt]->m_pFmmu[2].m_nLength=0x001;
		m_ppEcSlave[slvCnt]->m_pFmmu[2].m_nLgStartBit=0;
		m_ppEcSlave[slvCnt]->m_pFmmu[2].m_nLgStopBit=0x00;
		m_ppEcSlave[slvCnt]->m_pFmmu[2].m_nPhyStartAdr=0x80d;
		m_ppEcSlave[slvCnt]->m_pFmmu[2].m_nPhyStartBit=0;
		m_ppEcSlave[slvCnt]->m_pFmmu[2].m_nType=0x01;
		m_ppEcSlave[slvCnt]->m_pFmmu[2].m_nActive=0x01;
	}
}

void  CEcSimMaster::SendMail(
		uint8_t ecatindex,
		uint8_t slvNum,
		uint16_t index,
		uint8_t subindex,
		uint8_t len,
		uint32_t maildata){
	   int framelen=0;

		TETHERNET_88A4_MAX_FRAME ethFrame;
		ethFrame.E88A4.Reserved=0;
		ethFrame.E88A4.Type=1;
		ECAT_MAX_FRAME ecHdr;

		ecHdr.ECATHeader.cmd=5;
		ecHdr.ECATHeader.idx=ecatindex;
		ecHdr.ECATHeader.slAdr=m_ppEcSlave[slvNum]->m_nDlAddr;
		ecHdr.ECATHeader.ofsAdr=m_ppEcSlave[slvNum]->m_pSyncM[0].m_nPhyStart;
		ecHdr.ECATHeader.length=0x10;
		ecHdr.ECATHeader.next=1;
		ecHdr.ECATHeader.res=0;
		ecHdr.ECATHeader.irq=0;

		uint8_t * pData;
		pData=ethFrame.Data;
		memmove(pData,&ecHdr,10);
		pData=pData+10;
		framelen=framelen+10;
		COE_SDO_FRAME  *mailframe=(COE_SDO_FRAME*)malloc(sizeof(struct COE_SDO_FRAME));
		mailframe->MailHeader.Length=10;
		mailframe->MailHeader.Addr=0x0;
		mailframe->MailHeader.res=0;
		mailframe->MailHeader.Type=0x3;
		mailframe->MailHeader.Counter=mailcnt;

		mailframe->CoeHeader.PdoNo=0;
		mailframe->CoeHeader.res=0;
		mailframe->CoeHeader.Type=0x2;

		switch(len)
		{
		case 0:
			mailframe->SdoHeader.SodCmd=0x40;
			break;
		case 1:
			mailframe->SdoHeader.SodCmd=0x2f;
			break;
		case 2:
			mailframe->SdoHeader.SodCmd=0x2b;
			break;
		case 4:
			mailframe->SdoHeader.SodCmd=0x23;
		}

		mailframe->SdoHeader.Index=index;
		mailframe->SdoHeader.Sub=subindex;
		mailframe->SdoHeader.Data=maildata;
		memmove(pData,mailframe,16);
		pData=pData+16;
		framelen=framelen+16;
		for(int i=0;i<ecHdr.ECATHeader.length-16;i++)
		{
			pData[i]=0;
		}
		pData=pData+ecHdr.ECATHeader.length-16;
		framelen=framelen+ecHdr.ECATHeader.length-16;
		pData[0]=0x0;
		pData[1]=0x0;
		pData=pData+2;
		framelen=framelen+2;

		//framelen=framelen+2;

		ecHdr.ECATHeader.cmd=5;
		ecHdr.ECATHeader.idx=0x00;
		ecHdr.ECATHeader.ofsAdr=m_ppEcSlave[slvNum]->m_pSyncM[0].m_nPhyStart+m_ppEcSlave[slvNum]->m_pSyncM[0].m_nLength-2;
		ecHdr.ECATHeader.length=2;
		ecHdr.ECATHeader.next=0;
		memmove(pData,&ecHdr,10);
		pData=pData+10;
		framelen=framelen+10;
		pData[0]=0x0;
		pData[1]=0x0;
		pData=pData+2;
		framelen=framelen+2;
		pData[0]=0x0;
		pData[1]=0x0;
		pData=pData+2;
		framelen=framelen+2;

		ethFrame.E88A4.Length=framelen;
		int erro=0;
		while(erro<1)//若接收失败则重新发送
		{
			erro=m_pNetdev->SendPacket(framelen+2,(unsigned char *)&ethFrame);
			erro=m_npPcapdev->Receiver();
		}
		mailcnt++;
		if(mailcnt>=8)
			mailcnt=1;
}

void  CEcSimMaster::SendMail(
		uint8_t ecatindex,
		uint8_t slvcnt,
		uint8_t sdocmd,
		uint16_t index,
		uint8_t subindex,
		uint32_t len,
		uint64_t lmaildata){
	   int framelen=0;

		TETHERNET_88A4_MAX_FRAME ethFrame;
		ethFrame.E88A4.Reserved=0;
		ethFrame.E88A4.Type=1;
		ECAT_MAX_FRAME ecHdr;

		ecHdr.ECATHeader.cmd=5;
		ecHdr.ECATHeader.idx=ecatindex;
		ecHdr.ECATHeader.slAdr=m_ppEcSlave[slvcnt]->m_nDlAddr;
		ecHdr.ECATHeader.ofsAdr=m_ppEcSlave[slvcnt]->m_pSyncM[0].m_nPhyStart;
		ecHdr.ECATHeader.length=len+0x10;
		ecHdr.ECATHeader.next=1;
		ecHdr.ECATHeader.res=0;
		ecHdr.ECATHeader.irq=0;

		uint8_t * pData;
		pData=ethFrame.Data;
		memmove(pData,&ecHdr,10);
		pData=pData+10;
		framelen=framelen+10;
		COE_LSDO_FRAME  *mailframe=(COE_LSDO_FRAME*)malloc(sizeof(struct COE_LSDO_FRAME));
		mailframe->MailHeader.Length=10;
		mailframe->MailHeader.Addr=0x0;
		mailframe->MailHeader.res=0;
		mailframe->MailHeader.Type=0x3;
		mailframe->MailHeader.Counter=mailcnt;

		mailframe->CoeHeader.PdoNo=0;
		mailframe->CoeHeader.res=0;
		mailframe->CoeHeader.Type=0x2;

		mailframe->SdoHeader.SodCmd=sdocmd;
		mailframe->SdoHeader.Index=index;
		mailframe->SdoHeader.Sub=subindex;
		mailframe->SdoHeader.Len=len;
		mailframe->SdoHeader.MailData=lmaildata;
		memmove(pData,mailframe,16+len);
		pData=pData+16+len;
		framelen=framelen+16+len;
		for(int i=0;i<ecHdr.ECATHeader.length-16-len;i++)
		{
			pData[i]=0;
		}
		pData=pData+ecHdr.ECATHeader.length-16-len;
		framelen=framelen+ecHdr.ECATHeader.length-16-len;
		pData[0]=0x0;
		pData[1]=0x0;
		pData=pData+2;
		framelen=framelen+2;

		//framelen=framelen+2;
		ecHdr.ECATHeader.cmd=5;
		ecHdr.ECATHeader.idx=0x00;
		ecHdr.ECATHeader.ofsAdr=m_ppEcSlave[slvcnt]->m_pSyncM[0].m_nPhyStart+m_ppEcSlave[slvcnt]->m_pSyncM[0].m_nLength-2;
		ecHdr.ECATHeader.length=2;
		ecHdr.ECATHeader.next=0;
		memmove(pData,&ecHdr,10);
		pData=pData+10;
		framelen=framelen+10;
		pData[0]=0x0;
		pData[1]=0x0;
		pData=pData+2;
		framelen=framelen+2;
		pData[0]=0x0;
		pData[1]=0x0;
		pData=pData+2;
		framelen=framelen+2;

		ethFrame.E88A4.Length=framelen;
		int erro=0;
		while(erro<1)//若接收失败则重新发送
		{
			erro=m_pNetdev->SendPacket(framelen+2,(unsigned char *)&ethFrame);
			erro=m_npPcapdev->Receiver();
		}
		mailcnt++;
		if(mailcnt>=8)
			mailcnt=1;
}

void  CEcSimMaster::ReadMail(uint8_t slvNum,uint8_t index){
	   int framelen=0;
	   struct timeval tv;

		TETHERNET_88A4_MAX_FRAME ethFrame;
		ethFrame.E88A4.Reserved=0;
		ethFrame.E88A4.Type=1;
		ECAT_MAX_FRAME ecHdr;

		ecHdr.ECATHeader.cmd=4;
		ecHdr.ECATHeader.idx=index;
		ecHdr.ECATHeader.slAdr=m_ppEcSlave[slvNum]->m_nDlAddr;
		ecHdr.ECATHeader.ofsAdr=m_ppEcSlave[slvNum]->m_pSyncM[1].m_nPhyStart;
		ecHdr.ECATHeader.length=m_ppEcSlave[slvNum]->m_pSyncM[1].m_nLength;
		ecHdr.ECATHeader.next=0;
		ecHdr.ECATHeader.res=0;
		ecHdr.ECATHeader.irq=0;

		uint8_t * pData=ethFrame.Data;
		memcpy(pData,&ecHdr,10);
		pData=pData+10;
		framelen=framelen+10;
		for(int i=0;i<ecHdr.ECATHeader.length;i++)
		{
			pData[i]=0;
		}
		pData=pData+ecHdr.ECATHeader.length;
		framelen=framelen+ecHdr.ECATHeader.length;
		pData[0]=0;
		pData[1]=0;
		pData=pData+2;
		framelen=framelen+2;

		ethFrame.E88A4.Length=framelen;

		gettimeofday(&tv,NULL);
		__suseconds_t l_usectime=tv.tv_usec;
		__time_t l_sectime=tv.tv_sec;

	   for(int i=0;i<1;i++)
	   {
			gettimeofday(&tv,NULL);
			while(tv.tv_usec-l_usectime+(tv.tv_sec-l_sectime)*1000000<PDO_CYCL_TIME)
			{
				gettimeofday(&tv,NULL);
			}
			l_usectime=tv.tv_usec;
			l_sectime=tv.tv_sec;
			CheckSM(slvNum);
	   }

		int erro=0;
		while(erro<1)//若接收失败则重新发送
		{
			erro=m_pNetdev->SendPacket(framelen+2,(unsigned char *)&ethFrame);
			erro=m_npPcapdev->Receiver();
		}

}

void CEcSimMaster::InitConfig(){
	//数据链路控制
	uint32_t CtrData[m_nEcSlave];
	for(int j=0;j<m_nEcSlave;j++)
	{
		CtrData[j]=0x00070501;
	}

	uint8_t * pData=(uint8_t*)malloc(0x100);
	uint8_t ReadData[m_nEcSlave*20];
	for(int j=0;j<m_nEcSlave*20;j++)
	{
		ReadData[j]=0;
	}

	uint8_t  cmd=1;
	uint8_t  idx=0x82;
	uint16_t ofsAdr=0x0;
	uint16_t length=0x14;

	cmd=4;
	idx=0x82;
	ofsAdr=0x0110;
	length=2;
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,ReadData);

	cmd=5;
	ofsAdr=0x0100;
	length=4;
	SendECAT(4,idx,m_nEcSlave,ofsAdr,length,ReadData);
	memcpy(pData,&CtrData,length*m_nEcSlave);
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);
/*
	uint8_t ctr=0;
	cmd=1;
	ofsAdr=0x0102;
	length=1;
	memcpy(pData,&ctr,length*1);
	SendECAT(cmd,idx,1,ofsAdr,length,pData);

	ctr=0xfc;
	cmd=2;
	ofsAdr=0x0101;
	length=1;
	memcpy(pData,&ctr,length*1);
	SendECAT(cmd,idx,1,ofsAdr,length,pData);

	ctr=0x0;
	cmd=8;
	ofsAdr=0x0103;
	length=1;
	memcpy(pData,&ctr,length*1);
	SendECAT(cmd,idx,1,ofsAdr,length,pData);
*/
	cmd=4;
	ofsAdr=0x0200;
	length=2;
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,ReadData);

	cmd=4;
	ofsAdr=0x0204;
	length=4;
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,ReadData);

	cmd=4;
	ofsAdr=0x0204;
	length=4;
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,ReadData);

	uint16_t IrpData[m_nEcSlave];
	for(int j=0;j<m_nEcSlave;j++)
	{
		IrpData[j]=0x0004;
	}
	cmd=5;
	ofsAdr=0x0200;
	length=2;
	memcpy(pData,&IrpData,length*m_nEcSlave);
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);

	//EEPROM
	uint8_t EEPRomData[m_nEcSlave];
	for(int j=0;j<m_nEcSlave;j++)
	{
		EEPRomData[j]=0;
	}
	cmd=8;
	idx=0x82;
	ofsAdr=0x0500;
	length=1;
	memcpy(pData,&EEPRomData,length);
	SendECAT(cmd,idx,1,ofsAdr,length,pData);

	//EEPROMCTR
	cmd=7;
	ofsAdr=0x0502;
	length=2;
	SendECAT(cmd,idx,1,ofsAdr,length,ReadData);

	uint8_t EEPRomCTRData[m_nEcSlave][6];
	for(int j=0;j<m_nEcSlave;j++)
	{
		for(int i=0;i<6;i++)
			EEPRomCTRData[j][i]=0x0;
	}
	for(int j=0;j<m_nEcSlave;j++)
	{
		EEPRomCTRData[j][1]=0x01;
	}
	for(int j=0;j<m_nEcSlave;j++)
	{
		EEPRomCTRData[j][2]=0x08;
	}
	cmd=8;
	idx=0x82;
	ofsAdr=0x0502;
	length=6;
	memcpy(pData,&EEPRomCTRData,length);
	SendECAT(cmd,idx,1,ofsAdr,length,pData);

	cmd=7;
	ofsAdr=0x0502;
	length=2;
	SendECAT(cmd,idx,1,ofsAdr,length,ReadData);

	cmd=4;
	ofsAdr=0x0508;
	length=4;
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,ReadData);

	//EEPROMCTR
	cmd=7;
	ofsAdr=0x0502;
	length=2;
	SendECAT(cmd,idx,1,ofsAdr,length,ReadData);

	for(int j=0;j<m_nEcSlave;j++)
	{
		EEPRomCTRData[j][2]=0x0a;
	}
	cmd=8;
	idx=0x82;
	ofsAdr=0x0502;
	length=6;
	memcpy(pData,&EEPRomCTRData,length);
	SendECAT(cmd,idx,1,ofsAdr,length,pData);

	cmd=7;
	ofsAdr=0x0502;
	length=2;
	SendECAT(cmd,idx,1,ofsAdr,length,ReadData);

	cmd=4;
	ofsAdr=0x0508;
	length=4;
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,ReadData);

	//EEPROMCTR
	cmd=7;
	ofsAdr=0x0502;
	length=2;
	SendECAT(cmd,idx,1,ofsAdr,length,ReadData);

	for(int j=0;j<m_nEcSlave;j++)
	{
		EEPRomCTRData[j][2]=0x0c;
	}
	cmd=8;
	idx=0x82;
	ofsAdr=0x0502;
	length=6;
	memcpy(pData,&EEPRomCTRData,length);
	SendECAT(cmd,idx,1,ofsAdr,length,pData);

	cmd=7;
	ofsAdr=0x0502;
	length=2;
	SendECAT(cmd,idx,1,ofsAdr,length,ReadData);

	cmd=4;
	ofsAdr=0x0508;
	length=4;
	SendECAT(cmd,idx,1,ofsAdr,length,ReadData);

	///////////////////////////////////////140
	uint16_t PDIData[m_nEcSlave];
	for(int i=0;i<m_nEcSlave;i++)
		PDIData[i]=0;
	cmd=4;
	idx=0x82;
	ofsAdr=0x140;
	length=2;
	memcpy(pData,&PDIData,length*m_nEcSlave);
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);

	//DC CTR
	uint8_t DCCtrData[32];
	for(int j=0;j<0x20;j++)
			DCCtrData[j]=0;
	cmd=8;
	idx=0x82;
	ofsAdr=0x0910;
	length=0x20;
	memcpy(pData,&DCCtrData,length);
	SendECAT(cmd,idx,1,ofsAdr,length,pData);

	//DC SYNC
	uint8_t DCSyncData[m_nEcSlave];
	for(int j=0;j<m_nEcSlave;j++)
	{
		DCSyncData[j]=0x0;
	}
	cmd=8;
	idx=0x82;
	ofsAdr=0x0981;
	length=1;
	memcpy(pData,&DCSyncData,length);
	SendECAT(cmd,idx,1,ofsAdr,length,pData);

	//DC ?
	uint16_t DCData[m_nEcSlave];
	for(int j=0;j<m_nEcSlave;j++)
	{
		DCData[j]=0x1000;
	}
	cmd=8;
	idx=0x82;
	ofsAdr=0x0930;
	length=2;
	memcpy(pData,&DCData,length);
	SendECAT(cmd,idx,1,ofsAdr,length,pData);

	//DC deep
	uint16_t DCDeepData[m_nEcSlave];
	for(int j=0;j<m_nEcSlave;j++)
	{
		DCDeepData[j]=0x0c00;
	}
	cmd=8;
	idx=0x82;
	ofsAdr=0x0934;
	length=2;
	memcpy(pData,&DCDeepData,length);
	SendECAT(cmd,idx,1,ofsAdr,length,pData);

	//DataCTR
	uint8_t DataCTRData[m_nEcSlave];
	for(int j=0;j<m_nEcSlave;j++)
	{
		DataCTRData[j]=0;
	}
	cmd=8;
	idx=0x82;
	ofsAdr=0x0103;
	length=1;
	memcpy(pData,&DataCTRData,length);
	SendECAT(cmd,idx,1,ofsAdr,length,pData);

	//EEPROM
	for(int j=0;j<m_nEcSlave;j++)
	{
		EEPRomData[j]=0;
	}
	cmd=5;
	idx=0x82;
	ofsAdr=0x0500;
	length=1;
	memcpy(pData,&EEPRomData,length*m_nEcSlave);
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);

	//EEPROMCTR
	for(int j=0;j<m_nEcSlave;j++)
	{
		for(int i=0;i<6;i++)
			EEPRomCTRData[j][i]=0x0;
	}
	for(int j=0;j<m_nEcSlave;j++)
	{
		EEPRomCTRData[j][1]=0x01;
	}
	for(int j=0;j<m_nEcSlave;j++)
	{
		EEPRomCTRData[j][2]=0x08;
	}
	cmd=5;
	idx=0x82;
	ofsAdr=0x0502;
	length=6;
	memcpy(pData,&EEPRomCTRData,length*m_nEcSlave);
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);

	cmd=4;
	ofsAdr=0x0508;
	length=4;
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,ReadData);

	cmd=5;
	ofsAdr=0x0502;
	length=6;
	for(int j=0;j<m_nEcSlave;j++)
	{
		EEPRomCTRData[j][2]=0x0a;
	}
	memcpy(pData,&EEPRomCTRData,length*m_nEcSlave);
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);

	cmd=4;
	ofsAdr=0x0508;
	length=4;
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,ReadData);

	uint64_t IrpCtrData[m_nEcSlave];
	for(int j=0;j<m_nEcSlave;j++)
	{
		IrpCtrData[j]=0x00ffff00;
	}
	cmd=8;
	ofsAdr=0x0204;
	length=4;
	memcpy(pData,&IrpCtrData,length);
	SendECAT(cmd,idx,1,ofsAdr,length,pData);

	uint8_t fmmuData[256];
	for(int i=0;i<0x100;i++)
		fmmuData[i]=0;
	cmd=8;
	idx=0x82;
	ofsAdr=0x600;
	length=0x100;
	memcpy(pData,&fmmuData,length);
	SendECAT(cmd,idx,1,ofsAdr,length,fmmuData);

	uint8_t SMData[256];
	for(int i=0;i<0x100;i++)
		SMData[i]=0;
	cmd=8;
	idx=0x82;
	ofsAdr=0x800;
	length=0x100;
	memcpy(pData,&SMData,length);
	SendECAT(cmd,idx,1,ofsAdr,length,SMData);

	free(pData);
}

void CEcSimMaster::WatchDog(){
	uint16_t WatchDogData[m_nEcSlave];
	for(int j=0;j<m_nEcSlave;j++)
	{
		WatchDogData[j]=0x0990;
	}
	uint8_t * pData=(uint8_t*)malloc(0x20);

	uint8_t  cmd=5;
	uint8_t  idx=0x82;
	uint16_t ofsAdr=0x0400;
	uint16_t length=2;
	memcpy(pData,&WatchDogData,length*m_nEcSlave);
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);

	uint16_t CntData[m_nEcSlave];
	for(int j=0;j<m_nEcSlave;j++)
	{
		CntData[j]=0x03e8;
	}
	cmd=5;
	idx=0x82;
	ofsAdr=0x0420;
	length=2;
	memcpy(pData,&CntData,length*m_nEcSlave);
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);

	//EEPROM
		uint8_t EEPRomData[m_nEcSlave];
		for(int j=0;j<m_nEcSlave;j++)
		{
			EEPRomData[j]=0x1;
		}
		cmd=5;
		idx=0x82;
		ofsAdr=0x0500;
		length=1;
		memcpy(pData,&EEPRomData,length*m_nEcSlave);
		SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);

		free(pData);
}


void CEcSimMaster::CheckSM(uint8_t slvNum)
{
	int framelen=0;

			TETHERNET_88A4_MAX_FRAME ethFrame;
			ethFrame.E88A4.Reserved=0;
			ethFrame.E88A4.Type=1;
			ECAT_MAX_FRAME ecHdr;

			ecHdr.ECATHeader.cmd=7;
			ecHdr.ECATHeader.idx=0x82;
			ecHdr.ECATHeader.slAdr=m_ppEcSlave[0]->m_nDlAddr;
			ecHdr.ECATHeader.ofsAdr=0x80d;
			//ecHdr.ECATHeader.lgAdr=0x00080000;
			ecHdr.ECATHeader.length=1;
			ecHdr.ECATHeader.next=0;
			ecHdr.ECATHeader.res=0;
			ecHdr.ECATHeader.irq=0;

			uint8_t * pData=ethFrame.Data;
			memcpy(pData,&ecHdr,10);
			pData=pData+10;
			framelen=framelen+10;
			////////////////////////////////////////////////////Data
			for(int i=0;i<ecHdr.ECATHeader.length;i++)
			{
				pData[i]=0;
			}
			pData=pData+ecHdr.ECATHeader.length;
			framelen=framelen+ecHdr.ECATHeader.length;
			////////////////////////////////////////////////////WKC
			pData[0]=0;
			pData[1]=0;
			pData=pData+2;
			framelen=framelen+2;

			ethFrame.E88A4.Length=framelen;
			int erro=0;
			while(erro<1)//若接收失败则重新发送
			{
				erro=m_pNetdev->SendPacket(framelen+2,(unsigned char *)&ethFrame);
				erro=m_npPcapdev->Receiver();
			}
			//SYNCMANAGE *smdata=(SYNCMANAGE*)(pcap_packet+TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN+TETYPE_ECAT_HEADER_LEN+20);
			uint8_t * sm_status=(uint8_t*)(m_npPcapdev->pcap_packet+TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN+TETYPE_ECAT_HEADER_LEN);
			struct timeval tv;
			gettimeofday(&tv,NULL);
			__suseconds_t l_usectime=tv.tv_usec;
			__time_t l_sectime=tv.tv_sec;
			while(((*sm_status)&0x08)==0)
			{
				gettimeofday(&tv,NULL);
				if(tv.tv_usec-l_usectime+(tv.tv_sec-l_sectime)*1000000>100*PDO_CYCL_TIME)  //////////time out
				{
					cout << "slave"<<(int)slvNum<<"Time out: check mailbuffer"<<endl;
					ReMail(slvNum);
					break;
				}
				erro=m_pNetdev->SendPacket(framelen+2,(unsigned char *)&ethFrame);
				erro=m_npPcapdev->Receiver();
				sm_status=(uint8_t*)(m_npPcapdev->pcap_packet+TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN+TETYPE_ECAT_HEADER_LEN);
			}
}

void CEcSimMaster:: ReMail(uint8_t slvNum)
{
	uint8_t  cmd=5;
	uint8_t  idx=0x82;
	uint16_t ofsAdr=0x0806;
	uint16_t length=1;
	TETYPE_ECAT_HEADER *pECATframe;
	uint8_t sm=0;
	uint8_t * sm_pdi_status=&sm;

	//uint8_t * pData=(uint8_t*)malloc(length);
	uint8_t  pData=0x3;
	SSendECAT(cmd,idx,slvNum,ofsAdr,length,&pData);

	cmd=4;
	ofsAdr=0x80E;
	pData=0;
	SSendECAT(cmd,idx,slvNum,ofsAdr,length,&pData);
	pECATframe=(TETYPE_ECAT_HEADER*)(m_npPcapdev->pcap_packet+TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN);
    if(pECATframe->ofsAdr==0x80E){
    	sm_pdi_status=(uint8_t*)(m_npPcapdev->pcap_packet+TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN+TETYPE_ECAT_HEADER_LEN);
    }
	struct timeval tv;
	gettimeofday(&tv,NULL);
	__suseconds_t l_usectime=tv.tv_usec;
	__time_t l_sectime=tv.tv_sec;
	while(((*sm_pdi_status)&0x02)==0)
	{
		gettimeofday(&tv,NULL);
		if(tv.tv_usec-l_usectime+(tv.tv_sec-l_sectime)*1000000>100*PDO_CYCL_TIME)  //////////time out
		{
			cout << "slave"<<(int)slvNum<< "Time out: Recheck mailbuffer"<<endl;
			break;
		}
		SSendECAT(cmd,idx,slvNum,ofsAdr,length,&pData);
		pECATframe=(TETYPE_ECAT_HEADER*)(m_npPcapdev->pcap_packet+TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN);
	    if(pECATframe->ofsAdr==0x80E){
	    	sm_pdi_status=(uint8_t*)(m_npPcapdev->pcap_packet+TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN+TETYPE_ECAT_HEADER_LEN);
	    }

	}
}

void CEcSimMaster:: RealTimeSetting()
{
	struct timeval tv;
	gettimeofday(&tv,NULL);
	uint64_t realtime=tv.tv_sec*1000000000+tv.tv_usec*1000-0xD233C64749E062A;
	uint8_t slvcnt=0;
	uint8_t sdocmd=0x20;
	uint16_t index=0x2580;
	uint8_t subindex=4;
	uint32_t len=8;
	SendMail(0x81,slvcnt,sdocmd,index,subindex,len,realtime);
	ReadMail(slvcnt,0x82);

}

void CEcSimMaster:: SycnDC()
{
	//extern unsigned char* g_InputPacket;
	uint32_t *pDataback;
	uint32_t time1,time2[m_nEcSlave],time3[m_nEcSlave],time4,TimeDelay[m_nEcSlave];
	uint64_t TimeOffset[m_nEcSlave];
	int frmoffset=TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN+TETYPE_ECAT_HEADER_LEN;
	uint64_t SycnDCData[m_nEcSlave];

	for(int j=0;j<m_nEcSlave;j++)
	{
		SycnDCData[j]=0;
		time2[j]=0;
		time3[j]=0;
		TimeDelay[j]=0;
		TimeOffset[j]=0;
	}
	uint8_t * pData=(uint8_t*)malloc(8*SLAVE_CNT);

	uint8_t  cmd=14;
	uint8_t  idx=0x82;
	uint16_t ofsAdr=0x0910;
	uint16_t length=8;
	memcpy(pData,SycnDCData,length);
	SendECAT(cmd,idx,1,ofsAdr,length,pData);

	/////////////////////////////////////////////0x0900
	cmd=4;
	idx=0x82;
	ofsAdr=0x0900;
	length=8;
	memcpy(pData,&SycnDCData,length*m_nEcSlave);
	for(int i=0;i<20;i++){
		SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);
	}
	pDataback=(uint32_t *)(m_npPcapdev->pcap_packet+frmoffset);
	time1=*pDataback;
	pDataback=(uint32_t *)(m_npPcapdev->pcap_packet+frmoffset+4);
	time4=*pDataback;
	if(m_nEcSlave>1){
		for(int slvCnt=1;slvCnt<m_nEcSlave;slvCnt++)
		{
			frmoffset=TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN+TETYPE_ECAT_HEADER_LEN+(TETYPE_ECAT_HEADER_LEN+8+2)*slvCnt;
			pDataback=(uint32_t *)(m_npPcapdev->pcap_packet+frmoffset);
			time2[slvCnt]=*pDataback;
			pDataback=(uint32_t *)(m_npPcapdev->pcap_packet+frmoffset+4);
			time3[slvCnt]=*pDataback;

			TimeDelay[slvCnt]=((time4-time1)-(time3[slvCnt]-time2[slvCnt]))/2;
			TimeOffset[slvCnt]=time2[slvCnt]-time1-TimeDelay[slvCnt];
		}
	}

	cmd=5;
	idx=0x82;
	ofsAdr=0x0928;
	length=4;
	memcpy(pData,TimeDelay,length*m_nEcSlave);
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);
	ofsAdr=0x0920;
	length=8;
	memcpy(pData,TimeOffset,length*m_nEcSlave);
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);

	cmd=14;
	idx=0x82;
	ofsAdr=0x0910;
	length=8;
	memcpy(pData,&SycnDCData,length);
	if(m_nEcSlave>1){
		for(int i=0;i<1000;i++)
		{
			SendECAT(cmd,idx,1,ofsAdr,length,pData);
			usleep(50);
		}
	}

	cmd=5;
	idx=0x82;
	ofsAdr=0x092c;
	length=4;
	memcpy(pData,&SycnDCData,length*m_nEcSlave);
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);

	free(pData);
}

void CEcSimMaster:: SycnInit()
{
	//extern unsigned char* g_InputPacket;
	struct timeval tv;
	uint64_t *pDataback;
	int frmoffset=TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN+TETYPE_ECAT_HEADER_LEN;

	uint32_t SycnDCData[m_nEcSlave][2];
	for(int j=0;j<m_nEcSlave;j++)
	{
		SycnDCData[j][0]=1000*PDO_CYCL_TIME;
		SycnDCData[j][1]=0;
	}
	//uint32_t SycnDCData[2]={1000000,0};
	uint8_t * pData=(uint8_t*)malloc(8*SLAVE_CNT);

	///////////////////////////////////////////////////////////////sync type select
	uint8_t ecatindex;
	uint8_t slvcnt;
	uint8_t sdocmd;
	uint16_t index;
	uint8_t subindex;
	uint32_t maildata;

	ecatindex=0x82;
	index=0x1c32;

	for(slvcnt=0;slvcnt<m_nEcSlave;slvcnt++)
	{
		if(m_ppEcSlave[slvcnt]->dv_type==IO){
			cout<<"I/O SYCN TYPE"<<endl;
			subindex=1;
			maildata=0x2;
			SendMail(ecatindex,slvcnt,index,subindex,2,maildata);
			ReadMail(slvcnt,ecatindex+1);
			subindex=0xA;
			maildata=SycnDCData[slvcnt][0];
			cout<<"I/O SYCN TIME"<<endl;
			SendMail(ecatindex,slvcnt,index,subindex,4,maildata);
			ReadMail(slvcnt,ecatindex+1);
		}else
		{
			cout<<"SERVO SYCN TYPE"<<endl;
			subindex=1;
			maildata=0x2;
			SendMail(ecatindex,slvcnt,index,subindex,2,maildata);
			ReadMail(slvcnt,ecatindex+1);
		}
	}

	ecatindex=0x83;
	sdocmd=0x2b;
	index=0x1c33;
	for(slvcnt=0;slvcnt<m_nEcSlave;slvcnt++)
	{
		if(m_ppEcSlave[slvcnt]->dv_type==IO){
			cout<<"I/O SYCN TYPE"<<endl;
			subindex=1;
			maildata=0x2;
			SendMail(ecatindex,slvcnt,index,subindex,2,maildata);
			ReadMail(slvcnt,ecatindex+1);
			subindex=0xA;
			maildata=SycnDCData[slvcnt][0];
			cout<<"I/O SYCN TIME"<<endl;
			SendMail(ecatindex,slvcnt,index,subindex,4,maildata);
			ReadMail(slvcnt,ecatindex+1);
		}else{
			cout<<"SERVO SYCN TYPE"<<endl;
			subindex=1;
			maildata=0x2;
			SendMail(ecatindex,slvcnt,index,subindex,2,maildata);
			ReadMail(slvcnt,ecatindex+1);
		}
	}

	///////////////////////////////////////////////////////////////////SYNC0 Cycle Time
	uint8_t  cmd=5;
	uint8_t  idx=0x82;
	uint16_t ofsAdr=0x09A0;
	uint16_t length=8;
	memcpy(pData,&SycnDCData,length*m_nEcSlave);
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);

	/////////////////////////////////////////////////////////////////////Set start time
	cmd=4;
	idx=0x82;
	ofsAdr=0x0910;
	length=8;
	for(int i=0;i<length*m_nEcSlave;i++)
	{
		pData[i]=0;
	}

	for(int i=0;i<50;i++){
		SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);
	}
	pDataback=(uint64_t *)(m_npPcapdev->pcap_packet+frmoffset);
	uint64_t delaytime=80000000;
	uint64_t currenttime[m_nEcSlave];
	currenttime[0]=*pDataback;
	uint64_t time[m_nEcSlave];
	time[0]=currenttime[0]+delaytime;
	if(m_nEcSlave>1){
		for(slvcnt=0;slvcnt<m_nEcSlave;slvcnt++)
		{
			pDataback=(uint64_t *)(m_npPcapdev->pcap_packet+frmoffset+slvcnt*(TETYPE_ECAT_HEADER_LEN+2+length));
			currenttime[slvcnt]=*pDataback;
			time[slvcnt]=currenttime[slvcnt]+delaytime;
		}
	}

	length=2;
	ofsAdr=0x0982;
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);

	cmd=5;
	idx=0x82;
	ofsAdr=0x0990;
	length=8;
	memcpy(pData,time,length*m_nEcSlave);
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);

	//////////////////////////////////////////////////////////////////////Cycle Unit Enable
	uint8_t enable[m_nEcSlave];
	for(int j=0;j<m_nEcSlave;j++)
	{
		enable[j]=0x03;
	}
	cmd=5;
	idx=0x82;
	ofsAdr=0x0981;
	length=1;
	memcpy(pData,enable,length*m_nEcSlave);
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);

	///////////////////////////////////////////////////////////////////////waite
	cmd=4;
	idx=0x82;
	length=8;
	for(int i=0;i<length*m_nEcSlave;i++)
	{
		pData[i]=0;
	}
	gettimeofday(&tv,NULL);
	__suseconds_t l_usectime=tv.tv_usec;
	__time_t l_sectime=tv.tv_sec;
	do{
		ofsAdr=0x0990;
		SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);
		ofsAdr=0x0910;
		SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);
		pDataback=(uint64_t *)(m_npPcapdev->pcap_packet+frmoffset);
		currenttime[0]=*pDataback;
		if(m_nEcSlave>1){
			for(slvcnt=0;slvcnt<m_nEcSlave;slvcnt++)
			{
				pDataback=(uint64_t *)(m_npPcapdev->pcap_packet+frmoffset+slvcnt*(TETYPE_ECAT_HEADER_LEN+2+length));
				currenttime[slvcnt]=*pDataback;
			}
		}

		if(tv.tv_usec-l_usectime+(tv.tv_sec-l_sectime)*1000000>3500*PDO_CYCL_TIME)
		{
			cout << "Time out: sync init";
			break;
		}

		usleep(50);
		gettimeofday(&tv,NULL);
	}
	while(tv.tv_usec-l_usectime+(tv.tv_sec-l_sectime)*1000000<3000*PDO_CYCL_TIME);

	cmd=5;
	idx=0x82;
	ofsAdr=0x09a8;
	length=2;
	SendECAT(cmd,idx,m_nEcSlave,ofsAdr,length,pData);

	free(pData);
/*	*/
}

void CEcSimMaster::ServoParamInit(){

}

void CEcSimMaster::SendCyclicFrame(int cnt,uint8_t ECATIndex,RPDO_MAPPING *pRpdoData){
	int framelen=0;

	//cout<<"pRpdoData->TargetPos"<<pRpdoData->TargetPos<<endl;
		TETHERNET_88A4_MAX_FRAME ethFrame;
		ethFrame.E88A4.Reserved=0;
		ethFrame.E88A4.Type=1;
		ECAT_MAX_FRAME ecHdr;
		uint8_t * pData=ethFrame.Data;

		//////////////////////////////////////////////////////////////////////////////////////////////Status
		ecHdr.ECATHeader.res=0;
		ecHdr.ECATHeader.irq=0;
		ecHdr.ECATHeader.cmd=7;
		ecHdr.ECATHeader.idx=ECATIndex;
		ecHdr.ECATHeader.slAdr=0x0000;
		ecHdr.ECATHeader.ofsAdr=0x0130;
		ecHdr.ECATHeader.length=2;
		ecHdr.ECATHeader.next=1;

		memcpy(pData,&ecHdr,10);
		pData=pData+10;
		framelen=framelen+10;
		////////////////////////////////////////////////////Data
		for(int i=0;i<ecHdr.ECATHeader.length;i++)
		{
			pData[i]=0;
		}
		pData=pData+ecHdr.ECATHeader.length;
		framelen=framelen+ecHdr.ECATHeader.length;
		////////////////////////////////////////////////////WKC
		pData[0]=0;
		pData[1]=0;
		pData=pData+2;
		framelen=framelen+2;

		//////////////////////////////////////////////////////////////////////////////////////////////////////////输出FMMU logic
		int AddrOffset=0;
		for(int slvCnt=0;slvCnt<m_nEcSlave;slvCnt++)
		{
			AddrOffset=AddrOffset+m_ppEcSlave[slvCnt]->m_pSyncM[2].m_nLength;
		}
		//AddrOffset=m_nEcSlave*SM2_LENTH;
		LgDataLenth[0]=AddrOffset;
		ecHdr.ECATHeader.cmd=11;
		ecHdr.ECATHeader.idx=ECATIndex;
		ecHdr.ECATHeader.lgAdr=m_ppEcSlave[0]->m_pFmmu[0].m_nLgstartAdr;
		ecHdr.ECATHeader.length=AddrOffset;
		ecHdr.ECATHeader.next=1;
		memcpy(pData,&ecHdr,10);
		pData=pData+10;
		framelen=framelen+10;
		////////////////////////////////////////////////////Data
		memcpy(pData,pRpdoData,AddrOffset);
		pData=pData+ecHdr.ECATHeader.length;
		framelen=framelen+ecHdr.ECATHeader.length;
		////////////////////////////////////////////////////WKC
		pData[0]=0;
		pData[1]=0;
		pData=pData+2;
		framelen=framelen+2;

		//////////////////////////////////////////////////////////////////////////////////////////////////////////输入FMMU logic
		AddrOffset=0;
		for(int slvCnt=0;slvCnt<m_nEcSlave;slvCnt++)
		{
			AddrOffset=AddrOffset+m_ppEcSlave[slvCnt]->m_pSyncM[3].m_nLength;
		}
		//AddrOffset=m_nEcSlave*SM3_LENTH;
		LgDataLenth[1]=AddrOffset;
		ecHdr.ECATHeader.cmd=10;
		ecHdr.ECATHeader.idx=ECATIndex;
		ecHdr.ECATHeader.lgAdr=m_ppEcSlave[0]->m_pFmmu[1].m_nLgstartAdr;
		ecHdr.ECATHeader.length=AddrOffset;
		ecHdr.ECATHeader.next=1;
		memcpy(pData,&ecHdr,10);
		pData=pData+10;
		framelen=framelen+10;
		////////////////////////////////////////////////////Data
		for(int i=0;i<ecHdr.ECATHeader.length;i++){
			pData[i]=0;
		}
		pData=pData+ecHdr.ECATHeader.length;
		framelen=framelen+ecHdr.ECATHeader.length;
		////////////////////////////////////////////////////WKC
		pData[0]=0;
		pData[1]=0;
		pData=pData+2;
		framelen=framelen+2;

		///////////////////////////////////////////////////////////////////////////////////////////////////0x0900
		ecHdr.ECATHeader.cmd=4;
		ecHdr.ECATHeader.idx=ECATIndex;
		ecHdr.ECATHeader.slAdr=m_ppEcSlave[0]->m_nDlAddr;
		ecHdr.ECATHeader.ofsAdr=0x0990;
		ecHdr.ECATHeader.length=8;
		ecHdr.ECATHeader.next=1;
		memcpy(pData,&ecHdr,10);
		pData=pData+10;
		framelen=framelen+10;
		////////////////////////////////////////////////////Data
		for(int i=0;i<ecHdr.ECATHeader.length;i++)
		{
			pData[i]=0;
		}
		pData=pData+ecHdr.ECATHeader.length;
		framelen=framelen+ecHdr.ECATHeader.length;
		////////////////////////////////////////////////////WKC
		pData[0]=0;
		pData[1]=0;
		pData=pData+2;
		framelen=framelen+2;

		///////////////////////////////////////////////////////////////////////////////////////////////////0x0910
		ecHdr.ECATHeader.cmd=14;
		ecHdr.ECATHeader.idx=ECATIndex;
		ecHdr.ECATHeader.slAdr=m_ppEcSlave[0]->m_nDlAddr;
		ecHdr.ECATHeader.ofsAdr=0x0910;
		ecHdr.ECATHeader.length=8;
		ecHdr.ECATHeader.next=1;
		memcpy(pData,&ecHdr,10);
		pData=pData+10;
		framelen=framelen+10;
		////////////////////////////////////////////////////Data
		for(int i=0;i<ecHdr.ECATHeader.length;i++)
		{
			pData[i]=0;
		}
		pData=pData+ecHdr.ECATHeader.length;
		framelen=framelen+ecHdr.ECATHeader.length;
		////////////////////////////////////////////////////WKC
		pData[0]=0;
		pData[1]=0;
		pData=pData+2;
		framelen=framelen+2;

		///////////////////////////////////////////////////////////////////////////////////////////////////0x80000
		ecHdr.ECATHeader.cmd=10;
		ecHdr.ECATHeader.idx=ECATIndex;
		ecHdr.ECATHeader.lgAdr=0x80000;
		ecHdr.ECATHeader.length=1;
		ecHdr.ECATHeader.next=1;
		memcpy(pData,&ecHdr,10);
		pData=pData+10;
		framelen=framelen+10;
		////////////////////////////////////////////////////Data
		for(int i=0;i<ecHdr.ECATHeader.length;i++)
		{
			pData[i]=0;
		}
		pData=pData+ecHdr.ECATHeader.length;
		framelen=framelen+ecHdr.ECATHeader.length;
		////////////////////////////////////////////////////WKC
		pData[0]=0;
		pData[1]=0;
		pData=pData+2;
		framelen=framelen+2;

		///////////////////////////////////////////////////////////////////////////////////////////////////0x092c
		ecHdr.ECATHeader.cmd=7;
		ecHdr.ECATHeader.idx=ECATIndex;
		ecHdr.ECATHeader.slAdr=0;
		ecHdr.ECATHeader.ofsAdr=0x092c;
		ecHdr.ECATHeader.length=4;
		ecHdr.ECATHeader.next=0;
		memcpy(pData,&ecHdr,10);
		pData=pData+10;
		framelen=framelen+10;
		////////////////////////////////////////////////////Data
		for(int i=0;i<ecHdr.ECATHeader.length;i++)
		{
			pData[i]=0;
		}
		pData=pData+ecHdr.ECATHeader.length;
		framelen=framelen+ecHdr.ECATHeader.length;
		////////////////////////////////////////////////////WKC
		pData[0]=0;
		pData[1]=0;
		pData=pData+2;
		framelen=framelen+2;

		ethFrame.E88A4.Length=framelen;
		int erro=0;
	/*
		while(erro<1)//若接收失败则重新发送
		{
			erro=m_pNetdev->SendPacket(framelen+2,(unsigned char *)&ethFrame);
			erro=m_npPcapdev->Receiver();
		}
	*/
		erro=m_pNetdev->SendCyclPacket(framelen+2,(unsigned char *)&ethFrame);
/*		erro=m_npPcapdev->CyclReceiver(cnt);

		packetlen=m_npPcapdev->packetlen;
		memcpy(pcap_packet,m_npPcapdev->pcap_packet,packetlen);*/
}

void CEcSimMaster::SendCyclicFrame(int cnt,uint8_t ECATIndex,uint8_t *pRpdoData){
	int framelen=0;

	TETHERNET_88A4_MAX_FRAME ethFrame;
	ethFrame.E88A4.Reserved=0;
	ethFrame.E88A4.Type=1;
	ECAT_MAX_FRAME ecHdr;
	uint8_t * pData=ethFrame.Data;

	//////////////////////////////////////////////////////////////////////////////////////////////Status
	ecHdr.ECATHeader.res=0;
	ecHdr.ECATHeader.irq=0;
	ecHdr.ECATHeader.cmd=7;
	ecHdr.ECATHeader.idx=ECATIndex;
	ecHdr.ECATHeader.slAdr=0x0000;
	ecHdr.ECATHeader.ofsAdr=0x0130;
	ecHdr.ECATHeader.length=6;
	ecHdr.ECATHeader.next=1;

	memcpy(pData,&ecHdr,10);
	pData=pData+10;
	framelen=framelen+10;
	////////////////////////////////////////////////////Data
	for(int i=0;i<ecHdr.ECATHeader.length;i++)
	{
		pData[i]=0;
	}
	pData=pData+ecHdr.ECATHeader.length;
	framelen=framelen+ecHdr.ECATHeader.length;
	////////////////////////////////////////////////////WKC
	pData[0]=0;
	pData[1]=0;
	pData=pData+2;
	framelen=framelen+2;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////输出FMMU logic
	int AddrOffset=0;
	for(int slvCnt=0;slvCnt<m_nEcSlave;slvCnt++)
	{
		AddrOffset=AddrOffset+m_ppEcSlave[slvCnt]->m_pSyncM[2].m_nLength;
	}
	//AddrOffset=m_nEcSlave*SM2_LENTH;
	LgDataLenth[0]=AddrOffset;
	ecHdr.ECATHeader.cmd=11;
	ecHdr.ECATHeader.idx=ECATIndex;
	ecHdr.ECATHeader.lgAdr=m_ppEcSlave[0]->m_pFmmu[0].m_nLgstartAdr;
	ecHdr.ECATHeader.length=AddrOffset;
	ecHdr.ECATHeader.next=1;
	memcpy(pData,&ecHdr,10);
	pData=pData+10;
	framelen=framelen+10;
	////////////////////////////////////////////////////Data
	memcpy(pData,pRpdoData,AddrOffset);
	pData=pData+ecHdr.ECATHeader.length;
	framelen=framelen+ecHdr.ECATHeader.length;
	////////////////////////////////////////////////////WKC
	pData[0]=0;
	pData[1]=0;
	pData=pData+2;
	framelen=framelen+2;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////输入FMMU logic
	AddrOffset=0;
	for(int slvCnt=0;slvCnt<m_nEcSlave;slvCnt++)
	{
		AddrOffset=AddrOffset+m_ppEcSlave[slvCnt]->m_pSyncM[3].m_nLength;
	}
	//AddrOffset=m_nEcSlave*SM3_LENTH;
	LgDataLenth[1]=AddrOffset;
	ecHdr.ECATHeader.cmd=10;
	ecHdr.ECATHeader.idx=ECATIndex;
	ecHdr.ECATHeader.lgAdr=m_ppEcSlave[0]->m_pFmmu[1].m_nLgstartAdr;
	ecHdr.ECATHeader.length=AddrOffset;
	ecHdr.ECATHeader.next=1;
	memcpy(pData,&ecHdr,10);
	pData=pData+10;
	framelen=framelen+10;
	////////////////////////////////////////////////////Data
	for(int i=0;i<ecHdr.ECATHeader.length;i++){
		pData[i]=0;
	}
	pData=pData+ecHdr.ECATHeader.length;
	framelen=framelen+ecHdr.ECATHeader.length;
	////////////////////////////////////////////////////WKC
	pData[0]=0;
	pData[1]=0;
	pData=pData+2;
	framelen=framelen+2;

	///////////////////////////////////////////////////////////////////////////////////////////////////0x0900
	ecHdr.ECATHeader.cmd=4;
	ecHdr.ECATHeader.idx=ECATIndex;
	ecHdr.ECATHeader.slAdr=m_ppEcSlave[0]->m_nDlAddr;
	ecHdr.ECATHeader.ofsAdr=0x0990;
	ecHdr.ECATHeader.length=8;
	ecHdr.ECATHeader.next=1;
	memcpy(pData,&ecHdr,10);
	pData=pData+10;
	framelen=framelen+10;
	////////////////////////////////////////////////////Data
	for(int i=0;i<ecHdr.ECATHeader.length;i++)
	{
		pData[i]=0;
	}
	pData=pData+ecHdr.ECATHeader.length;
	framelen=framelen+ecHdr.ECATHeader.length;
	////////////////////////////////////////////////////WKC
	pData[0]=0;
	pData[1]=0;
	pData=pData+2;
	framelen=framelen+2;

	///////////////////////////////////////////////////////////////////////////////////////////////////0x0910
	ecHdr.ECATHeader.cmd=14;
	ecHdr.ECATHeader.idx=ECATIndex;
	ecHdr.ECATHeader.slAdr=m_ppEcSlave[0]->m_nDlAddr;
	ecHdr.ECATHeader.ofsAdr=0x0910;
	ecHdr.ECATHeader.length=8;
	ecHdr.ECATHeader.next=1;
	memcpy(pData,&ecHdr,10);
	pData=pData+10;
	framelen=framelen+10;
	////////////////////////////////////////////////////Data
	for(int i=0;i<ecHdr.ECATHeader.length;i++)
	{
		pData[i]=0;
	}
	pData=pData+ecHdr.ECATHeader.length;
	framelen=framelen+ecHdr.ECATHeader.length;
	////////////////////////////////////////////////////WKC
	pData[0]=0;
	pData[1]=0;
	pData=pData+2;
	framelen=framelen+2;

	///////////////////////////////////////////////////////////////////////////////////////////////////0x80000
	ecHdr.ECATHeader.cmd=10;
	ecHdr.ECATHeader.idx=ECATIndex;
	ecHdr.ECATHeader.lgAdr=0x80000;
	ecHdr.ECATHeader.length=1;
	ecHdr.ECATHeader.next=1;
	memcpy(pData,&ecHdr,10);
	pData=pData+10;
	framelen=framelen+10;
	////////////////////////////////////////////////////Data
	for(int i=0;i<ecHdr.ECATHeader.length;i++)
	{
		pData[i]=0;
	}
	pData=pData+ecHdr.ECATHeader.length;
	framelen=framelen+ecHdr.ECATHeader.length;
	////////////////////////////////////////////////////WKC
	pData[0]=0;
	pData[1]=0;
	pData=pData+2;
	framelen=framelen+2;

	///////////////////////////////////////////////////////////////////////////////////////////////////0x092c
	ecHdr.ECATHeader.cmd=7;
	ecHdr.ECATHeader.idx=ECATIndex;
	ecHdr.ECATHeader.slAdr=0;
	ecHdr.ECATHeader.ofsAdr=0x092c;
	ecHdr.ECATHeader.length=4;
	ecHdr.ECATHeader.next=0;
	memcpy(pData,&ecHdr,10);
	pData=pData+10;
	framelen=framelen+10;
	////////////////////////////////////////////////////Data
	for(int i=0;i<ecHdr.ECATHeader.length;i++)
	{
		pData[i]=0;
	}
	pData=pData+ecHdr.ECATHeader.length;
	framelen=framelen+ecHdr.ECATHeader.length;
	////////////////////////////////////////////////////WKC
	pData[0]=0;
	pData[1]=0;
	pData=pData+2;
	framelen=framelen+2;

	ethFrame.E88A4.Length=framelen;
	int erro=0;
/*
	while(erro<1)//若接收失败则重新发送
	{
		erro=m_pNetdev->SendPacket(framelen+2,(unsigned char *)&ethFrame);
		erro=m_npPcapdev->Receiver();
	}
*/
	erro=m_pNetdev->SendCyclPacket(framelen+2,(unsigned char *)&ethFrame);
/*	erro=m_npPcapdev->CyclReceiver(cnt);

	packetlen=m_npPcapdev->packetlen;
	memcpy(pcap_packet,m_npPcapdev->pcap_packet,packetlen);*/

}

void CEcSimMaster::Read220(){
	//uint8_t * pData=(uint8_t*)malloc(0x4);
	uint8_t ReadData[4];
	for(int i=0;i<4;i++)
	{
		ReadData[i]=0;
	}
	uint8_t  cmd=4;
	uint8_t  idx=0x82;
	uint16_t ofsAdr=0x220;
	uint16_t length=0x4;
	SendECAT(cmd,idx,1,ofsAdr,length,ReadData);
}
void CEcSimMaster::PdoAssign(uint8_t slvnum,uint16_t index,uint16_t obj_assign[],uint8_t num){
	uint8_t subindex;
	uint8_t ecatindex=0x82;
	uint32_t maildata;

	subindex=0;
	maildata=0x00;
	SendMail(ecatindex,slvnum,index,subindex,1,maildata);
	ReadMail(slvnum,ecatindex+1);

	for(int i=0;i<num;i++){
		subindex=1+i;
		maildata=obj_assign[i];
		SendMail(ecatindex,slvnum,index,subindex,2,maildata);
		ReadMail(slvnum,ecatindex+1);
	}

	subindex=0;
	maildata=num;
	SendMail(ecatindex,slvnum,index,subindex,1,maildata);
	ReadMail(slvnum,ecatindex+1);
}

void CEcSimMaster::PdoMapping(uint8_t slvnum,uint16_t index,uint32_t obj_mapping[],uint8_t num){
	uint8_t subindex;
	uint8_t ecatindex=0x82;
	uint32_t maildata;

	subindex=0;
	maildata=0x00;
	SendMail(ecatindex,slvnum,index,subindex,1,maildata);
	ReadMail(slvnum,ecatindex+1);

	for(int i=0;i<num;i++){
		subindex=1+i;
		maildata=obj_mapping[i];
		SendMail(ecatindex,slvnum,index,subindex,4,maildata);
		ReadMail(slvnum,ecatindex+1);
	}

	subindex=0;
	maildata=num;
	SendMail(ecatindex,slvnum,index,subindex,1,maildata);
	ReadMail(slvnum,ecatindex+1);
}

/*void CEcSimMaster::Delay(uint32_t delaytime_us){
	struct timeval tv;
	gettimeofday(&tv,NULL);
	__suseconds_t l_usectime=tv.tv_usec;
	__time_t l_sectime=tv.tv_sec;
	while((tv.tv_usec-l_usectime+(tv.tv_sec-l_sectime)*1000000)<delaytime_us){
		gettimeofday(&tv,NULL);
	}
}*/

void CEcSimMaster::CoEWrite(uint8_t slvnum,COE_WRITE *writedata){
	uint8_t ecatindex=0x82;
	SendMail(ecatindex,slvnum,writedata->index,writedata->subindex,writedata->len,writedata->data);
	ReadMail(slvnum,ecatindex+1);
}

uint32_t	CEcSimMaster::CoERead(uint8_t slvnum,COE_WRITE *writedata)
{
	uint32_t coe_read=0;
	uint8_t ecatindex=0x82;
	int frmoffset=TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN;
	SendMail(ecatindex,slvnum,writedata->index,writedata->subindex,writedata->len,writedata->data);
	ReadMail(slvnum,ecatindex+1);
	TETYPE_ECAT_HEADER *pECATframe;
	pECATframe=(TETYPE_ECAT_HEADER*)(m_npPcapdev->pcap_packet+frmoffset);
	while(pECATframe->ofsAdr!=m_ppEcSlave[slvnum]->m_pSyncM[1].m_nPhyStart)
	{
		m_npPcapdev->Receiver();
		pECATframe=(TETYPE_ECAT_HEADER*)(m_npPcapdev->pcap_packet+frmoffset);
	}
	frmoffset+= TETYPE_ECAT_HEADER_LEN+MAIL_HEADER_LEN+COE_HEADER_LEN;
	SDO_HEADER *psdo;
	psdo=(SDO_HEADER *)(m_npPcapdev->pcap_packet+frmoffset);
	if((psdo->Index==writedata->index)&&(psdo->Sub==writedata->subindex))
	{
		coe_read=psdo->Data;
	}
	return coe_read;
}

void CEcSimMaster::ReadServoParam(){

}

void CEcSimMaster::DeviceControl(){


}


