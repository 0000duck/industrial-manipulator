/*
 * CEcNetDevice.h
 *
 *  Created on: 2015年12月16日
 *      Author: root
 */

#ifndef CECNETDEVICE_H_
#define CECNETDEVICE_H_
#include <libnet.h>

class CEcNetDevice {
public:
	CEcNetDevice();
	virtual ~CEcNetDevice();
	int Opendev();
	int SendPacket(unsigned int Len,unsigned char* packet);
	int SendCyclPacket(unsigned int Len,unsigned char* packet);
	void Closedev();

private:

	char *dev;
	char errbuf[100];
	unsigned char desMac[6];
	unsigned char srcMac[6];
	libnet_t *plibnet_app;
	libnet_ptag_t  ptag;
	libnet_ptag_t  ptag0;
	struct timeval tv;
	__suseconds_t l_usectime;
	__time_t l_sectime;
};

#endif /* CECNETDEVICE_H_ */
