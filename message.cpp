#include "message.h"
#include <iostream>
#include <iomanip>

void FillMsg(Msg& m)
{
	m.useTimeStamp = ID_TIMESTAMP;
	m.timeStamp = RakNet::GetTime();
	m.typeId = ID_MY_MSG;
	m.forward = 0;
	m.right = 0;
	m.up = 0;
}


void Dump(Msg& m)
{
	std::cout << "t:" << m.timeStamp << "(" << (int)m.forward << " " << (int)m.right << " " << (int)m.up << ")\n";
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
