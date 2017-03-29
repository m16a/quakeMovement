#include "message.h"
#include <iostream>
#include <iomanip>

void FillMsg(SvMsg& m)
{
	m.useTimeStamp = ID_TIMESTAMP;
	m.serverTime = RakNet::GetTime();
	m.typeId = ID_SV_MSG;
	m.forward = 0;
	m.right = 0;
	m.up = 0;
}


void Dump(SvMsg& m)
{
	std::cout << "t:" << m.serverTime << "(" << (int)m.forward << " " << (int)m.right << " " << (int)m.up << ")\n";
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
