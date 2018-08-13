/*
 * CEcNetDevice.cpp
 *
 *  Created on: 2015年12月16日
 *      Author: root
 */


#include "CEcNetDevice.h"
#include "EthService.h"

CEcNetDevice::CEcNetDevice() {
	// TODO Auto-generated constructor stub
	dev="eth0";
    unsigned char desAddr[6]={0xff,0xff,0xff,0xff,0xff,0xff};
	unsigned char srcAddr[6]={0x54,0xEE,0x75,0x42,0xAE,0x3F};
	for(int i=0;i<6;i++)
	{
          desMac[i]=desAddr[i];
	      srcMac[i]=srcAddr[i];
	}
	 plibnet_app=new libnet_t;
	 ptag=0;
	 ptag0=0;

	 gettimeofday(&tv,NULL);
	 l_usectime=tv.tv_usec;
	 l_sectime=tv.tv_sec;
}

CEcNetDevice::~CEcNetDevice() {
	// TODO Auto-generated destructor stub
	delete plibnet_app;
}

int CEcNetDevice::Opendev(){
	plibnet_app=libnet_init(LIBNET_LINK_ADV,dev,errbuf);
	while(plibnet_app==NULL)
	{
		dev="eth1";
/*		if(dev=="eth0"){
			cout<<"eth0 Init failed!"<<endl;
			dev="eth1";
		}else{
			cout<<"eth1 Init failed!"<<endl;
			dev="eth0";
		}*/
		plibnet_app=libnet_init(LIBNET_LINK_ADV,dev,errbuf);
/*		printf("Sentpockets:Init failed!\n");
		return 1;*/
		usleep(40000);
	}
	//plibnet_app=libnet_init(LIBNET_LINK_ADV,dev,errbuf);
	return 0;
}

int CEcNetDevice::SendPacket(unsigned int Len,unsigned char* packet){
	//plibnet_app=libnet_init(LIBNET_LINK_ADV,dev,errbuf);
	ptag0=libnet_build_ethernet(
			desMac,
			srcMac,
			0x88A4,
			packet,
			Len,
			plibnet_app,
			 ptag);
	 ptag=ptag0;

		int error=libnet_write(plibnet_app);
		return error;
}

int CEcNetDevice::SendCyclPacket(unsigned int Len,unsigned char* packet){
	//plibnet_app=libnet_init(LIBNET_LINK_ADV,dev,errbuf);
	ptag0=libnet_build_ethernet(
			desMac,
			srcMac,
			0x88A4,
			packet,
			Len,
			plibnet_app,
			 ptag);
	 ptag=ptag0;

/*	 gettimeofday(&tv,NULL);
	 while((tv.tv_usec-l_usectime+(tv.tv_sec-l_sectime)*1000000)<PDO_CYCL_TIME)
		{
			gettimeofday(&tv,NULL);
		}
	 if(tv.tv_usec-l_usectime+(tv.tv_sec-l_sectime)*1000000>PDO_CYCL_TIME+500){
		 printf("Cycle time:%u\n",(tv.tv_usec-l_usectime)+(tv.tv_sec-l_sectime)*1000000);
	 }
	 l_usectime=tv.tv_usec;
	 l_sectime=tv.tv_sec;*/

		int error=libnet_write(plibnet_app);
		return error;
}

void CEcNetDevice::Closedev()
{
	libnet_destroy(plibnet_app);
}
