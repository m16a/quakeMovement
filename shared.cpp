#include "shared.h"
#include <ctime>
#include <iostream>

const int kServerPort =  60005;
const char* kHost = "127.0.0.1";


float GetCurrTime()
{
	return clock() / float(CLOCKS_PER_SEC);
}

unsigned char GetPacketIdentifier(RakNet::Packet *p)
{
	if ((unsigned char)p->data[0] == ID_TIMESTAMP)
	{
		std::cout << "my message\n";
		return (unsigned char) p->data[sizeof(RakNet::MessageID) + sizeof(RakNet::Time)];
	}
	else
		return (unsigned char) p->data[0];
}

