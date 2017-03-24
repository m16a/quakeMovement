#include <iostream>

#include "shared.h"

int main()
{
	std::cout << "Client\n";

	RakNet::RakPeerInterface* peer = RakNet::RakPeerInterface::GetInstance();
	assert(peer);

	peer->SetOccasionalPing(true);

	RakNet::SocketDescriptor* p_SD = new RakNet::SocketDescriptor();
	peer->Startup(1, p_SD, 1);

	peer->Connect(kHost, kServerPort, 0, 0);

	float lastSentTime = GetCurrTime();
	while (true)
	{
		const float currT = GetCurrTime();
		if (currT < lastSentTime + 0.05)
			continue;

		lastSentTime = currT;
		Msg m;
		FillMsg(m);
		Dump(m);

#if USE_BIT_STREAM 
		RakNet::BitStream myBitStream;
		myBitStream.Write(m.useTimeStamp);
		myBitStream.Write(m.timeStamp);
		myBitStream.Write(m.typeId);
		myBitStream.Write(m.x);
		myBitStream.Write(m.y);
		myBitStream.Write(m.z);
		peer->Send(&myBitStream, HIGH_PRIORITY, RELIABLE, 0, RakNet::UNASSIGNED_RAKNET_GUID, true);
#else
		ReverseTimeStamp(m);
		peer->Send(reinterpret_cast<char*>(&m), sizeof(Msg), HIGH_PRIORITY, RELIABLE, 0, RakNet::UNASSIGNED_RAKNET_GUID, true);
#endif

		std::cout << "sent t:" << currT << "\n";
		

		RakNet::Packet *packet;
		for (packet=peer->Receive(); packet; peer->DeallocatePacket(packet), packet=peer->Receive())
		{
			std::cout << "reading\t";
			unsigned char type = GetPacketIdentifier(packet);
			switch (type)
			{
				case ID_MY_MSG: 
					{
						Msg* m = reinterpret_cast<Msg*>(packet->data);
						assert(packet->length == sizeof(Msg));

						Dump(*m);
						break;
					}
				case ID_CONNECTION_REQUEST_ACCEPTED:
					std::cout << "connection accepted\n";
					break;
				case ID_CONNECTION_ATTEMPT_FAILED:
					std::cout << "connection failed\n";
					break;	
				default:
					std::cout << "unhandled type:" << (int)type << "\n";
					if (packet->data)
						std::cout << "\t data:" << packet->data << "\n";
					break;

			}
		}
	}

	return 0;	
}
