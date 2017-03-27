#ifndef _SHARED_H_
#define _SHARED_H_

#include "raknet/RakPeerInterface.h"
#include "raknet/RakNetTypes.h"
#include "raknet/MessageIdentifiers.h"

#include "message.h"

#define USE_BIT_STREAM 0

extern const int kServerPort;
extern const char* kHost;

float GetCurrTime();
unsigned char GetPacketIdentifier(RakNet::Packet *p);

struct usrcmd
{
	RakNet::Time serverTime;
	signed char forward, right, jump;
};

struct pstate
{
	usrcmd cmd;
	float vec[3];
};

#endif //_SHARED_H_

