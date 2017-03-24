#include "message.h"
#include <iostream>
#include <iomanip>

#include "raknet/GetTime.h"

void FillMsg(Msg& m)
{
	m.useTimeStamp = ID_TIMESTAMP;
	m.timeStamp = RakNet::GetTime();
	m.typeId = ID_MY_MSG;
	m.x = 42;
	m.y = 0;
	m.z = 0;
}


void Dump(Msg& m)
{
	std::cout << "t:" << m.timeStamp << "(" << m.x << " " << m.y << " " << m.z << ")\n";
	/*
	for (int i=0; i < sizeof(Msg); ++i)
	{
		std::cout << " " << std::hex << std::setfill('0') << std::setw(2) << *(reinterpret_cast<unsigned char*>(&m)+i);
	}
	std::cout << "\n";
	*/
}

void ReverseTimeStamp(Msg& m)
{
	RakNet::BitStream::ReverseBytesInPlace(reinterpret_cast<unsigned char*>(&m.timeStamp), sizeof(RakNet::Time));
}
