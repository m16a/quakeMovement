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

#endif //_SHARED_H_

