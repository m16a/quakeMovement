#include <iostream>
#include "raknet/RakPeerInterface.h"
#include "raknet/RakNetTypes.h"
#include "raknet/MessageIdentifiers.h"

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <sys/time.h>

#include "shared.h"

#ifndef DRAWSTUFF_TEXTURE_PATH
#define DRAWSTUFF_TEXTURE_PATH "textures"
#endif

// some constants

#define NUM 100			// number of bodies
#define NUMJ 9			// number of joints
#define SIDE (0.1)		// side length of a box
#define MASS (1.0)		// mass of a box
#define RADIUS (0.1732f)	// sphere radius
#define DENSITY (5.0)		// density of all objects

const int kMaxConnectionsAllowed = 2;
const int kMaxPlayersPerServer = 2;

RakNet::RakPeerInterface* gPeer = 0;

double gLastSentTime = 0; 

pstate gServerPState;

static bool gFlying = true;

struct MyObject {
  dBodyID body;			// the body
  dGeomID geom;		// geometry representing this body
};

static MyObject obj[NUM];

static dWorldID world=0;
static dSpaceID space;
static dJointID joint[NUMJ];
static dJointGroupID contactgroup;

static dGeomID  ground;

static int num = 28;

static float gViewRot[3] = {0.0f, 0.0f, 0.0f};

void drawGeom(dGeomID g, const dReal *pos, const dReal *R)
{
    if (!g)
        return;
    if (!pos)
        pos = dGeomGetPosition(g);
    if (!R)
        R = dGeomGetRotation(g);

    int type = dGeomGetClass(g);
    if (type == dBoxClass) {

        dVector3 sides;
        dGeomBoxGetLengths (g,sides);
        dsDrawBox(pos,R,sides);
    }
}

// create the test system
void createTest()
{
  dMass m;
  int i,j;
  if (world) dWorldDestroy (world);

  space = dHashSpaceCreate(0);
  world = dWorldCreate();
  contactgroup = dJointGroupCreate(0);

	//test box
	obj[0].body = dBodyCreate(world);
	dMassSetBoxTotal(&m, 1, 10000000, 10000000, 10000000);//disable rotation
	obj[0].geom = dCreateBox(space, SIDE, SIDE, SIDE);
  dBodySetPosition(obj[0].body, 0, 0, SIDE / 2.0f);
	dBodySetMass(obj[0].body, &m);

	dGeomSetBody(obj[0].geom, obj[0].body);

	//ground contact check ray
	obj[1].geom = dCreateRay(space, SIDE / 2.0f + 0.01);
	dGeomRaySet(obj[1].geom, 0,0,0.2, 0,0,-SIDE/2.0f - 0.001);

	//test map 
	obj[2].geom = dCreateBox(space, 1, 1, 0.5);
  dGeomSetPosition(obj[2].geom, 5, 0, 0.25);

	obj[3].geom = dCreateBox(space, 1, 1, 0.5);
  dGeomSetPosition(obj[3].geom, 3, 0, 0.25);

	for (int i=0; i<5; ++i)
		for (int j = 0; j<5; ++j)
		{
			obj[5*i+j+4].geom = dCreateBox(space, 0.03, 0.03, 0.1);
			dGeomSetPosition(obj[5*i+j+4].geom, 5 + i*0.25, 2 + j*0.25, 0.05);
		}
}

// start simulation - set viewpoint
static void start()
{
  dAllocateODEDataForThread(dAllocateMaskAll);
}

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  assert(o1);
  assert(o2);

  if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
  {
      fprintf(stderr,"testing space %p %p\n", (void*)o1, (void*)o2);
    // colliding a space with something
    dSpaceCollide2(o1,o2,data,&nearCallback);
    // Note we do not want to test intersections within a space,
    // only between spaces.
    return;
  }

//  fprintf(stderr,"testing geoms %p %p\n", o1, o2);

	//skip collsion btw main box and ground ray
	if (o1 == obj[0].geom && o2 == obj[1].geom ||
		o1 == obj[1].geom && o2 == obj[0].geom	)
		return;

  const int N = 32;
  dContact contact[N];
  int n = dCollide (o1,o2,N,&(contact[0].geom),sizeof(dContact));
  if (n > 0) 
  {
		if (o1 == obj[1].geom || o2 == obj[1].geom) 
		{
			gFlying = false;
			return;
		}
			
    for (int i=0; i<n; i++) 
    {
      contact[i].surface.slip1 = 0.7;
      contact[i].surface.slip2 = 0.7;
      contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
      contact[i].surface.bounce = 0.1;
      contact[i].surface.mu = dInfinity;
      contact[i].surface.soft_erp = 0.96;
      contact[i].surface.soft_cfm = 0.04;
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c,
		    dGeomGetBody(contact[i].geom.g1),
		    dGeomGetBody(contact[i].geom.g2));
    }
  }
}

static void step (float step, usrcmd c)
{
	//std::cout << "step\n";
	gFlying = true;

	
	//std::cout << "f:" << gFlying << "\n";
	const dReal* pos = dBodyGetPosition(obj[0].body);
	dGeomRaySet(obj[1].geom, pos[0], pos[1], pos[2], 0,0,-SIDE/2.0f - 0.001);

	dSpaceCollide (space,0,&nearCallback);
	
	if (!gFlying)
	{
		const dReal* v = dBodyGetLinearVel(obj[0].body);

		float zVel = v[2];
		if (c.jump)
			zVel = kPlayerJumpVelZ;

		dBodySetLinearVel(obj[0].body, c.forward / (100.0f/kPlayerMaxSpeed), c.right / (100.0f/kPlayerMaxSpeed), zVel);
	}

	dWorldQuickStep (world, step);
	dJointGroupEmpty (contactgroup);

	//const dReal* rot = dBodyGetQuaternion(obj[0].body);
	const dReal* w = dBodyGetAngularVel(obj[0].body);
	const dReal* v = dBodyGetLinearVel(obj[0].body);
	

	// remove all contact joints
	dJointGroupEmpty(contactgroup);


  float offset = 0.5;
  float dir2d[2] = {cos(c.yaw / 180.0f * M_PI), sin(c.yaw / 180.0f * M_PI)};
  float xyz[3] = {pos[0] - offset*dir2d[0], pos[1] - offset*dir2d[1], pos[2]+0.2};
	float rot[3] = {c.yaw, c.pitch, 0};
  dsSetViewpoint(xyz, rot);

}

static void simLoop (int pause)
{
#if LOG_PACKETS 
	std::cout << "[FRAME]\n";
#endif
	dsSetTexture(DS_WOOD);

	dsSetColor(1,0,0);
	drawGeom(obj[0].geom,0,0);
	for (int i=1; i<num; i++)
	{
		dsSetColor(1,1,0);
		drawGeom(obj[i].geom,0,0);
	}

	timeval tv;
	gettimeofday(&tv, 0);
	const double currT = tv.tv_sec + (double) tv.tv_usec / 1000000.0 ;
	{
		if (currT < gLastSentTime + 0.05)
			return;

		dsSetInfoToDraw(1.0f/(currT - gLastSentTime), int(kPacketLoss * 100), kPacketExtraLagMS);
		gLastSentTime = currT;
		

#if LOG_PACKETS 
		std::cout << "check mail box\n";
#endif

		RakNet::Packet *packet;
		for (packet=gPeer->Receive(); packet; gPeer->DeallocatePacket(packet), packet=gPeer->Receive())
		{
			unsigned char type = GetPacketIdentifier(packet);
			switch (type)
			{
				case ID_SV_MSG: 
					{
						SvMsg m;
						SvMsg* pm = reinterpret_cast<SvMsg*>(packet->data);
						assert(packet->length == sizeof(SvMsg));
						m = *pm;
						ReverseTimeStamp(m);
						m.serverTime += gPeer->GetClockDifferential(packet);
						
						//int  msec = m.serverTime - gServerPState.lastCommandTime;
						int  msec = m.cmd.cmdTime; 
#if LOG_PACKETS 
						std::cout << "receive. to simulate:" << msec << " "; Dump(m);
#endif
						if (!msec)
						{

							std::cout << "ALERT! cmd diff time is 0\n";

							if (m.serverTime > 0)
								gServerPState.lastCommandTime = m.serverTime;
							continue;
						}
						float sec = msec / 1000.f;
						
						//obtain input velocity from packet
						usrcmd& c = m.cmd;
						
						//we already simulate this cmd
						if (c.serverTime <= gServerPState.lastCommandTime)
							continue;

						//simulation
						step(sec, c);

						gServerPState.lastCommandTime = m.serverTime;

						const dReal* pos = dBodyGetPosition(obj[0].body);
						gServerPState.pos[0] = pos[0];
						gServerPState.pos[1] = pos[1];
						gServerPState.pos[2] = pos[2];

						const dReal* v = dBodyGetLinearVel(obj[0].body);
						gServerPState.vel[0] = v[0];
						gServerPState.vel[1] = v[1];
						gServerPState.vel[2] = v[2];

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
		//send back state
		ClMsg m;
		m.typeId = ID_CL_MSG;
		m.state = gServerPState;
		if (gServerPState.lastCommandTime > 0)
		{
#if LOG_PACKETS 
			std::cout << "send state: "; Dump(m);
#endif
			gPeer->Send(reinterpret_cast<char*>(&m), sizeof(ClMsg), HIGH_PRIORITY, UNRELIABLE_SEQUENCED, 0, RakNet::UNASSIGNED_RAKNET_GUID, true);
		}
	}
}


int main (int argc, char **argv)
{
	std::cout << "Server\n";

	gPeer = RakNet::RakPeerInterface::GetInstance();
	//gPeer->ApplyNetworkSimulator(1, 100, 0);
	assert(gPeer);


	gPeer->SetOccasionalPing(true);

	RakNet::SocketDescriptor* p_SD = new RakNet::SocketDescriptor(kServerPort,0);

	gPeer->ApplyNetworkSimulator(kPacketLoss, kPacketExtraLagMS, 0);
	gPeer->Startup(kMaxConnectionsAllowed, p_SD, 1);
	gPeer->SetMaximumIncomingConnections(kMaxPlayersPerServer);

  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = 0;
  fn.commandRelease = 0;
  fn.mouseMove = 0;
	
  fn.stop = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

  dInitODE2(0);
  //dRandSetSeed (time(0));
  dRandSetSeed (1);
  createTest();

  dWorldSetGravity (world,0,0,kGravityZ);

  ground = dCreatePlane (space,0,0,1,0);
  // run simulation
  dsSimulationLoop (argc,argv, 50, 50, 800,600,&fn);


  dJointGroupDestroy(contactgroup);
  dWorldDestroy (world);
  dCloseODE();

	return 0;	
}
