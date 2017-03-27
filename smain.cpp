#include <iostream>
#include "raknet/RakPeerInterface.h"
#include "raknet/RakNetTypes.h"
#include "raknet/MessageIdentifiers.h"

#include "shared.h"

const int kMaxConnectionsAllowed = 2;
const int kMaxPlayersPerServer = 2;


int main()
{
	std::cout << "Server\n";

	RakNet::RakPeerInterface* peer = RakNet::RakPeerInterface::GetInstance();
	assert(peer);

	peer->SetOccasionalPing(true);

	RakNet::SocketDescriptor* p_SD = new RakNet::SocketDescriptor(kServerPort,0);
	peer->Startup(kMaxConnectionsAllowed, p_SD, 1);
	peer->SetMaximumIncomingConnections(kMaxPlayersPerServer);

	float lastSentTime = GetCurrTime();
	while (true)
	{
		const float currT = GetCurrTime();
		if (currT < lastSentTime + 0.05)
			continue;

		RakNet::Packet *packet;
		for (packet=peer->Receive(); packet; peer->DeallocatePacket(packet), packet=peer->Receive())
		{
			unsigned char type = GetPacketIdentifier(packet);
			switch (type)
			{
				case ID_MY_MSG: 
					{
#if USE_BIT_STREAN
						RakNet::Msg m;
						BitStream myBitStream(packet->data, packet->length, false); // The false is for efficiency so we don't make a copy of the passed data
						myBitStream.Read(m.useTimeStamp);
						myBitStream.Read(m.timeStamp);
						myBitStream.Read(m.typeId);
						myBitStream.Read(m.x);
						myBitStream.Read(m.y);
						myBitStream.Read(m.z);
						m.useTimeStamp += peer->GetClockDifferential(packet);
						Dump(m);
						
#else
						Msg* m = reinterpret_cast<Msg*>(packet->data);
						assert(packet->length == sizeof(Msg));
						ReverseTimeStamp(*m);
						m->timeStamp += peer->GetClockDifferential(packet);
						Dump(*m);
#endif
						//std::cout << "my_msg:" << packet->length << ":" << sizeof(Msg) << "\n";

						break;
					}
				case ID_NEW_INCOMING_CONNECTION:
					std::cout << "new connection\n";
					break;
				default:
					std::cout << "unhandled type:" << (int)type << "\n";
					std::cout << "\t data:" << packet->data << "\n";
					break;

			}
		} 
	}

	return 0;	
}
