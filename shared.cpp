#include "shared.h"
#include <iostream>

#include <iostream>
#include <iomanip>
#include <ctime>


const int kServerPort =  60005;
const char* kHost = "127.0.0.1";
extern const float kPacketLoss = 0.0f;
extern const int kPacketExtraLagMS = 0;


float gPlayerMaxSpeed = 1.0f;

float GetCurrTime()
{
	return clock() / float(CLOCKS_PER_SEC);
}

unsigned char GetPacketIdentifier(RakNet::Packet *p)
{
	if ((unsigned char)p->data[0] == ID_TIMESTAMP)
	{
		return (unsigned char) p->data[sizeof(RakNet::MessageID) + sizeof(RakNet::Time)];
	}
	else
		return (unsigned char) p->data[0];
}

void Dump(const pstate& s)
{
	std::cout << "pstate t:" << s.lastCommandTime << " pos:(" << s.pos[0] << ", " << s.pos[1] << ", " << s.pos[2] << ") vel:(" << s.vel[0] << ", " << s.vel[1] << ", " << s.vel[2] << ")\n";
}

void FillMsg(SvMsg& m)
{
	m.useTimeStamp = ID_TIMESTAMP;
	m.serverTime = RakNet::GetTime();
	m.typeId = ID_SV_MSG;
	m.cmd.forward = 0;
	m.cmd.right = 0;
	m.cmd.jump = 0;
}


void Dump(SvMsg& m)
{
	std::cout << "t:" << m.serverTime << " ";
	Dump(m.cmd);
	/*
	for (int i=0; i < sizeof(Msg); ++i)
	{
		std::cout << " " << std::hex << std::setfill('0') << std::setw(2) << *(reinterpret_cast<unsigned char*>(&m)+i);
	}
	std::cout << "\n";
	*/
}

void ReverseTimeStamp(SvMsg& m)
{
	RakNet::BitStream::ReverseBytesInPlace(reinterpret_cast<unsigned char*>(&m.serverTime), sizeof(RakNet::Time));
}

void Dump(ClMsg& m)
{
	Dump(m.state);
}

void Dump(const usrcmd& c)
{
	std::cout << "t:" << c.serverTime << "(" << (int)c.forward << " " << (int)c.right << " " << (int)c.jump << ") rot(" << c.yaw << ", " << c.pitch <<")\n";
}
