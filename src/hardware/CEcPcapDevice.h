/*
 * CEcPcapDevice.h
 *
 *  Created on: 2015年12月16日
 *      Author: root
 */

#ifndef CECPCAPDEVICE_H_
#define CECPCAPDEVICE_H_

#include <pcap.h>
#include <string.h>

class CEcPcapDevice {
public:
	CEcPcapDevice();
	virtual ~CEcPcapDevice();
	int Opendev();
	int Receiver();
	int CyclReceiver(int cnt);
	void CheckRecvFrame(u_char * packet);
	void Closedev();
	static void PcapCallBack(u_char * arg, const struct pcap_pkthdr * pkthdr, const u_char * packet);
public:
	unsigned char * pcap_packet;
	int packetlen;
private:
	//char  * devStr;
	pcap_t * device;

};



#endif /* CECPCAPDEVICE_H_ */
