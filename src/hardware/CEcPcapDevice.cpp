/*
 * CEcPcapDevice.cpp
 *
 *  Created on: 2015年12月16日
 *      Author: root
 */


#include "CEcPcapDevice.h"
#include "EthService.h"

CEcPcapDevice::CEcPcapDevice() {
	// TODO Auto-generated constructor stub
	//devStr=new char[PCAP_ERRBUF_SIZE];
	pcap_packet=(unsigned char*)malloc(ETHERNET_MAX_FRAME_LEN);
	packetlen=0;
	device=(pcap_t*)malloc(sizeof(pcap_t*));//new pcap_t;
}

CEcPcapDevice::~CEcPcapDevice() {
	// TODO Auto-generated destructor stub
	//delete device;
	//delete devStr;
}
int CEcPcapDevice::Opendev(){
	  char errBuf[PCAP_ERRBUF_SIZE];
	  /* get a device */
	  char *devStr = pcap_lookupdev(errBuf);

	  if(devStr)
	  {
	    printf("device: %s\n", devStr);
	  }
	  else
	  {
	    printf("error: %s\n", errBuf);
	    return 1;
	  }
	  /* open a device*/
	  device = pcap_open_live(devStr, 1518, 1, 1, errBuf);//超时时间1ms

	  if(!device)
	  {
	    printf("error: pcap_open_live(): %s\n", errBuf);
	    return 1;
	  }

	  //编译过滤器
	  struct bpf_program filter;
	  //char filter_app[]="ether src 56:EE:75:42:AE:3F";
	  char filter_app[]="not ether src 54:EE:75:42:AE:3F and ether proto 0x88a4";
	 // char filter_app[]="ether proto 0x88a4";
	  pcap_compile(device, &filter, filter_app, 1, 0);
	  pcap_setfilter(device, &filter);
	  return 0;
}

int CEcPcapDevice::CyclReceiver(int cnt){
	  int id = 0;
	  //struct pcap_pkthdr * pkthdr;
	  //u_char * packet;//=(u_char*)malloc(ETHERNET_MAX_FRAME_LEN);
	  //packet=(u_char*)pcap_next(device,pkthdr);

	  int erro=pcap_dispatch(device, cnt,PcapCallBack, (u_char*)this);
	 // int erro=pcap_loop(device, 1,CallBack, (u_char*)&id);
	  //CheckRecvFrame(g_InputPacket);
/*	  extern unsigned char* g_InputPacket;
	  extern int g_InputSize;
	  packetlen=g_InputSize;
	  memcpy(pcap_packet,g_InputPacket,packetlen);*/
	  //CheckRecvFrame(pcap_packet);

	  return erro;
}

int CEcPcapDevice::Receiver(){
	  int id = 0;
	  //struct pcap_pkthdr * pkthdr;
	  //u_char * packet;//=(u_char*)malloc(ETHERNET_MAX_FRAME_LEN);
	  //packet=(u_char*)pcap_next(device,pkthdr);
	 // int erro=pcap_dispatch(device, 1,CallBack, (u_char*)&id);
		 /*struct timeval tv;
		gettimeofday(&tv,NULL);
		printf("pcaptime1:%u:%u\n",tv.tv_sec,tv.tv_usec);*/
	  int erro=pcap_loop(device, 1,PcapCallBack, (u_char*)this);
		/*gettimeofday(&tv,NULL);
		printf("pcaptime2:%u:%u\n",tv.tv_sec,tv.tv_usec);*/
	  //CheckRecvFrame(g_InputPacket);
/*	  extern unsigned char* g_InputPacket;
	  extern int g_InputSize;
	  packetlen=g_InputSize;
	  memcpy(pcap_packet,g_InputPacket,packetlen);*/
	  //CheckRecvFrame(pcap_packet);

	  return 1;
}
void CEcPcapDevice::PcapCallBack(u_char * arg, const struct pcap_pkthdr * pkthdr, const u_char * packet){
	CEcPcapDevice *device=(CEcPcapDevice*)arg;
	memcpy(device->pcap_packet,packet,pkthdr->len);
	device->packetlen=pkthdr->len;
}

void CEcPcapDevice::CheckRecvFrame(u_char * packet){

	/*printf("Packet length: %d\n", pkthdr->len);
	  printf("Number of bytes: %d\n", pkthdr->caplen);
	  printf("Recieved time: %s", ctime((const time_t *)&pkthdr->ts.tv_sec));*/
	  //extern int g_InputSize;
	  int frmoffset=0;

	  TETHERNET_HEADER *pEth_Header=(TETHERNET_HEADER*)malloc(sizeof(struct TETHERNET_HEADER));
	  pEth_Header=(TETHERNET_HEADER*)packet;
		printf("Destination is:");
		for(int i=0;i<6;i++)
			printf(" %02x",pEth_Header->Destination.Adrr[i]);
		printf("\nSource is:");
		for(int i=0;i<6;i++)
		    printf(" %02x",pEth_Header->Source.Adrr[i]);
	  printf("\nFrameType: %x\n",pEth_Header->FrameType);
	  if(pEth_Header->FrameType==0xa488)
	  {
		  TETYPE_ECAT_HEADER *pECATframe;
		  frmoffset+=TETHERNET_HEADER_LEN+ETYPE_88A4_HEADER_LEN;
	      pECATframe=(TETYPE_ECAT_HEADER*)(packet+frmoffset);
		  printf("Cmd: %d\n",pECATframe->cmd);
		  printf("Index: %x\n",pECATframe->idx);
	      if((pECATframe->cmd>9)&&(pECATframe->cmd<13))
	      {
			  printf("Logic addr: %x\n",pECATframe->lgAdr);
	      }
	      else
	      {
			  printf("Slave addr: %x\n",pECATframe->slAdr);
			  printf("Offset addr: %x\n",pECATframe->ofsAdr);
	      }
		  printf("Length: %d\n",pECATframe->length);
		  printf("Interrupt: %x\n",pECATframe->irq);
		  frmoffset+= TETYPE_ECAT_HEADER_LEN;

		  if((pECATframe->ofsAdr==0x1c00)||(pECATframe->ofsAdr==0x1800)||(pECATframe->ofsAdr==0x1000)||(pECATframe->ofsAdr==0x1200))
		  {
			  MAIL_HEADER *pmailheader;
			  pmailheader=(MAIL_HEADER *)(packet+frmoffset);
			  printf("Mailbox:\n");
			  printf("Len:%x\n",pmailheader->Length);
			  printf("Address:%x\n",pmailheader->Addr);
			  printf("Type:%x\n",pmailheader->Type);
			  printf("Counter:%x\n",pmailheader->Counter);
			  frmoffset+= MAIL_HEADER_LEN;

			  COE_HEADER *pcoeheader;
			  pcoeheader=(COE_HEADER *)(packet+frmoffset);
			  printf("Number:%x\n",pcoeheader->PdoNo);
			  printf("Type:%x\n",pcoeheader->Type);
			  frmoffset+= COE_HEADER_LEN;

			  SDO_HEADER *psdo;
			  psdo=(SDO_HEADER *)(packet+frmoffset);
			  printf("SodCmd:%x\n",psdo->SodCmd);
			  printf("Index:%x\n",psdo->Index);
			  printf("SubIndex:%x\n",psdo->Sub);
			  printf("Data:%x",psdo->Data);
			  frmoffset+= SDO_HEADER_LEN;
		  }
		  else
		  {
			  printf("Data:");
			  uint8_t * pData;
			  for(int i=0;i<pECATframe->length;i++)
			  {
				  pData=(uint8_t *)(packet+frmoffset);
				  printf(" %02x",*pData);
				  frmoffset+=1;
			  }
		  }

		  uint16_t *pWKC;
		  pWKC=(uint16_t *)(packet+frmoffset);
		  printf("\nWKC: %x\n",*pWKC);
	  }

	  int i;
	  for(i=0; i<packetlen; ++i)
	  {
	    printf(" %02x", packet[i]);
	    if( (i + 1) % 16 == 0 )
	    {
	      printf("\n");
	    }
	  }
	  printf("\n\\\\\\\\\*********************************************************************************************\\\\\\\\\n");
}


void CEcPcapDevice::Closedev() {
	pcap_close(device);
}


