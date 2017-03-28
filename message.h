#ifndef _MESSAGE_H_
#define _MESSAGE_H_

#include "raknet/RakNetTypes.h"
#include "raknet/MessageIdentifiers.h"
#include "raknet/BitStream.h"
#include "raknet/GetTime.h"

enum 
{
	ID_MY_MSG = ID_USER_PACKET_ENUM+1
};

#pragma pack(push, 1)
struct Msg 
{
	unsigned char useTimeStamp; // Assign ID_TIMESTAMP to this
	RakNet::Time serverTime; // Put the system time in here returned by RakNet::GetTime()
	unsigned char typeId; // You should put here an enum you defined after the last one defined in MessageIdentifiers.h, lets say ID_SET_TIMED_MINE
	signed char forward, right, up;

	//RakNet::NetworkID networkId; // NetworkID of the mine, used as a common method to refer to the mine on different computers
	//RakNet::SystemAddress systemAddress; // The SystenAddress of the player that owns the mine
};
#pragma pack(pop)

void FillMsg(Msg& m);
void Dump(Msg& m);
void ReverseTimeStamp(Msg& m);
#endif //_MESSAGE_H_
