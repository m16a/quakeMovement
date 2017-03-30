#ifndef _SHARED_H_
#define _SHARED_H_


#include "raknet/RakPeerInterface.h"
#include "raknet/RakNetTypes.h"
#include "raknet/MessageIdentifiers.h"
#include "raknet/GetTime.h"
#include "raknet/BitStream.h"


#define LOG_PACKETS 0

enum 
{
	ID_SV_MSG = ID_USER_PACKET_ENUM+1,
	ID_CL_MSG 
};

#define LOG_PREDICTION 0

extern const int kServerPort;
extern const char* kHost;
extern const float kPacketLoss;
extern const int kPacketExtraLagMS;

extern const float kPlayerMaxSpeed;
extern const float kGravityZ;
extern const float kPlayerJumpVelZ;

float GetCurrTime();
unsigned char GetPacketIdentifier(RakNet::Packet *p);

struct usrcmd
{
	RakNet::Time serverTime;
	signed char forward, right, jump;
	float yaw, pitch;
};

void Dump(const usrcmd& c);

struct pstate
{
	//usrcmd cmd;
	RakNet::Time lastCommandTime;
	float pos[3];
	float vel[3];
};

void Dump(const pstate& s);

//message from client to server
#pragma pack(push, 1)
struct SvMsg 
{
	unsigned char useTimeStamp; // Assign ID_TIMESTAMP to this
	RakNet::Time serverTime; // Put the system time in here returned by RakNet::GetTime()
	unsigned char typeId; // You should put here an enum you defined after the last one defined in MessageIdentifiers.h, lets say ID_SET_TIMED_MINE
	//signed char forward, right, up;
	usrcmd cmd;

	//RakNet::NetworkID networkId; // NetworkID of the mine, used as a common method to refer to the mine on different computers
	//RakNet::SystemAddress systemAddress; // The SystenAddress of the player that owns the mine
};
#pragma pack(pop)

void FillMsg(SvMsg& m);
void Dump(SvMsg& m);
void ReverseTimeStamp(SvMsg& m);


//message from server to client
#pragma pack(push, 1)
struct ClMsg 
{
	unsigned char typeId; 
	pstate state;
	//RakNet::NetworkID networkId; // NetworkID of the mine, used as a common method to refer to the mine on different computers
	//RakNet::SystemAddress systemAddress; // The SystenAddress of the player that owns the mine
};
#pragma pack(pop)

void Dump(ClMsg& m);
#endif //_SHARED_H_

