#include <iostream>

#include "shared.h"


#include <time.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <cassert>
#include <sys/time.h>

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#ifndef DRAWSTUFF_TEXTURE_PATH
#define DRAWSTUFF_TEXTURE_PATH "textures"
#endif

// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif


// some constants

#define NUM 100			// number of bodies
#define NUMJ 9			// number of joints
#define SIDE (0.1)		// side length of a box
#define MASS (1.0)		// mass of a box
#define RADIUS (0.1732f)	// sphere radius
#define DENSITY (5.0)		// density of all objects

RakNet::RakPeerInterface* gPeer = 0;

const float kStepSize = 0.01f;
double gLastSentTime = 0; 

RakNet::Time gLastCommandTime = 0;

struct MyObject {
  dBodyID body;			// the body
  dGeomID geom;		// geometry representing this body
};

static MyObject obj[NUM];

// dynamics and collision objects

static dWorldID world=0;
static dSpaceID space;
static dJointID joint[NUMJ];
static dJointGroupID contactgroup;

static dGeomID  ground;

static int num = 28;

static float gViewRot[3] = {0.0f, 0.0f, 0.0f};
static bool gFlying = true;

int gCmdIndex = 0;
int gLastSentCmdIndex = 0;

#define MAX_COMMANDS 128
#define CMD_MASK (MAX_COMMANDS - 1)
usrcmd commands[MAX_COMMANDS];

pstate gPlayerState;
pstate gPlayerAckState;

enum 
{
	eMoveFrwd = 1,
	eMoveBck = 1 << 1,
	eMoveLeft = 1 << 2,
	eMoveRight = 1 << 3,
	eMoveJump = 1 << 4

};

static int gMoveFlags = 0;

void drawGeom(dGeomID g, const dReal *pos, const dReal *R)
{
    if (!g)
        return;
    if (!pos)
        pos = dGeomGetPosition(g);
    if (!R)
        R = dGeomGetRotation(g);

    int type = dGeomGetClass(g);
    if (type == dBoxClass)
		{
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

	gPlayerState.pos[0] = 0;
	gPlayerState.pos[1] = 0;
	gPlayerState.pos[2] = SIDE / 2.0f;

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
void createCMD()
{
	const dReal* pos = dBodyGetPosition(obj[0].body);

	int vec[2] = {0,0};
	if (gMoveFlags & eMoveFrwd)
		vec[0] += 1;
	if (gMoveFlags & eMoveBck)
		vec[0] -= 1;

	if (gMoveFlags & eMoveRight)
		vec[1] += 1;
	if (gMoveFlags & eMoveLeft)
		vec[1] -= 1;

	{
		float dir2d[2] = {cos(gViewRot[0] / 180.0f * M_PI), sin(gViewRot[0] / 180.0f * M_PI)};
		
		//std::cout << "dir2d:"<< dir2d[0] << "-" << dir2d[1] << "\n";
		float res[2] = {vec[0] * dir2d[0] + vec[1] * dir2d[1], vec[0] * dir2d[1] - vec[1] * dir2d[0]};
		//std::cout << "move:"<< vec[0] << "-" << vec[1] << "\n";
		if (vec[0] || vec[1])
		{	
				float res_norm = sqrt(res[0]*res[0] + res[1]*res[1]);
				res[0] /= res_norm;
				res[1] /= res_norm;
		}

		float speed = kPlayerMaxSpeed;
		//if (!gFlying)
		{
			const dReal* v = dBodyGetLinearVel(obj[0].body);
			float resVel[3] = {speed * res[0], speed * res[1], v[2]};
			//std::cout << "clVel: " << resVel[0] << " " << resVel[1] << " " << resVel[2] << "\n"; 
	//		dBodySetLinearVel(obj[0].body, resVel[0], resVel[1], resVel[2]);

			usrcmd& c = commands[(++gCmdIndex) & CMD_MASK];
			c.serverTime = RakNet::GetTime();
			c.forward = static_cast<signed char>(resVel[0] * 100.0f/kPlayerMaxSpeed);
			c.right = static_cast<signed char>(resVel[1] * 100.0f/kPlayerMaxSpeed);
			c.jump = 0;
			
			if (gCmdIndex == 1)
			{
				c.cmdTime = 16;
			}	
			else
			{
				c.cmdTime = c.serverTime - commands[(gCmdIndex-1) & CMD_MASK].serverTime;
			}
				
			if (gMoveFlags & eMoveJump)
			{
				c.jump = 1;
				//std::cout << "create jump cmd\n";
			}
			
			gMoveFlags &= ~eMoveJump;

			c.yaw = gViewRot[0];
			c.pitch = gViewRot[1];
		}
	}

}

static void step (float step, usrcmd c)
{
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
		{
			zVel = kPlayerJumpVelZ;
			//std::cout << "jumping\n";
		}

		dBodySetLinearVel(obj[0].body, c.forward / (100.0f/kPlayerMaxSpeed), c.right / (100.0f/kPlayerMaxSpeed), zVel);
	}

	dWorldQuickStep (world, step);
	dJointGroupEmpty (contactgroup);

	const dReal* rot = dBodyGetQuaternion(obj[0].body);
	const dReal* w = dBodyGetAngularVel(obj[0].body);
	const dReal* v = dBodyGetLinearVel(obj[0].body);
	
	// remove all contact joints
	dJointGroupEmpty(contactgroup);

}

static void simLoop (int pause)
{
#if LOG_PACKETS
	std::cout << "[FRAME]\n";
#endif

	gFlying = true;
	createCMD();
	
	pstate oldState = gPlayerState;
	gPlayerState =  gPlayerAckState;

  dBodySetPosition(obj[0].body, gPlayerState.pos[0], gPlayerState.pos[1], gPlayerState.pos[2]);
	dBodySetLinearVel(obj[0].body, gPlayerState.vel[0], gPlayerState.vel[1], gPlayerState.vel[2]);

	dsSetColorAlpha(0,1,0, 0.7);
	drawGeom(obj[0].geom,0,0);

	bool noprediction = 0;
	if (!noprediction)
	{
#if LOG_PREDICTION 
		std::cout << "\t prediction start:"; Dump(gPlayerState);
		std::cout << "\t currPos:"; Dump(oldState);
#endif 
		int i = gCmdIndex - MAX_COMMANDS + 1;
		for (; i <= gCmdIndex; ++i)
		{
			usrcmd& c = commands[i & CMD_MASK];
				
			if (c.serverTime <= gPlayerState.lastCommandTime)
				continue;

			//std::cout << "\t prediction inprog:"; Dump(oldState);
			//std::cout << "\t prediction inprog:"; Dump(gPlayerState);
			if (oldState.lastCommandTime == gPlayerState.lastCommandTime)
			{
				//check prediction here
				float dist = (oldState.pos[0] - gPlayerState.pos[0])*(oldState.pos[0] - gPlayerState.pos[0])+(oldState.pos[1] - gPlayerState.pos[1])*(oldState.pos[1] - gPlayerState.pos[1])+(oldState.pos[2] - gPlayerState.pos[2])*(oldState.pos[2] - gPlayerState.pos[2]);
				dist = sqrt(dist);
				if (dist > 0.1)
					std::cout << "[PREDICTION ERROR]:" << dist << "\n"; 
			}

			int msecs = c.serverTime - gPlayerState.lastCommandTime;
#if LOG_PREDICTION 
		std::cout << "\t dt:" << msecs << " left:" << (gCmdIndex - i) << " "; Dump(c);
#endif 
			if (!msecs)
				continue;

			dsSetColorAlpha(0,0,1, 0.3);
			drawGeom(obj[0].geom,0,0);

			float sec = msecs / 1000.f;
			step(sec, c);
			gPlayerState.lastCommandTime += msecs;
			const dReal* pos = dBodyGetPosition(obj[0].body);
			gPlayerState.pos[0] = pos[0];
			gPlayerState.pos[1] = pos[1];
			gPlayerState.pos[2] = pos[2];

			const dReal* vel = dBodyGetLinearVel(obj[0].body);
			gPlayerState.vel[0] = vel[0];
			gPlayerState.vel[1] = vel[1];
			gPlayerState.vel[2] = vel[2];
#if LOG_PREDICTION 
		std::cout << "\t\t"; Dump(gPlayerState);
#endif 
		}
	}
	else 
	{
		usrcmd empty;
		empty.forward = empty.right = 0;
		step(0.001, empty);
	}

  float offset = 0.5;
	const dReal* pos = dBodyGetPosition(obj[0].body);
  float dir2d[2] = {cos(gViewRot[0] / 180.0f * M_PI), sin(gViewRot[0] / 180.0f * M_PI)};
  float xyz[3] = {pos[0] - offset*dir2d[0], pos[1] - offset*dir2d[1], pos[2]+0.2};
  dsSetViewpoint(xyz,gViewRot);
	
	//drawing
	dsSetTexture(DS_WOOD);
	dsSetColor(1,0,0);
	drawGeom(obj[0].geom,0,0);
	for (int i=1; i<num; i++)
	{
		dsSetColor(1,1,0);
		drawGeom(obj[i].geom,0,0);
	}

	// NETWORK STAF
	{
		timeval tv;
		gettimeofday(&tv, 0);
		const double currT = tv.tv_sec + (double) tv.tv_usec / 1000000.0 ;
		RakNet::Time currStateT = gPlayerState.lastCommandTime;
		//if (currT < gLastSentTime + 0.05)
			//return;
		double rt = currT-gLastSentTime;
		double st = (currStateT-gLastCommandTime) / 1000.0f;


		if (st > 2 * rt)
			std::cout << "frame:" << rt<< " simulated:" << st<< "\n";

		dsSetInfoToDraw(1.0f/(currT - gLastSentTime), int(kPacketLoss * 100), kPacketExtraLagMS);

		gLastSentTime = currT;
		gLastCommandTime =  gPlayerState.lastCommandTime;

		if (gLastSentCmdIndex < 10)
			gLastSentCmdIndex = 0;
		else 
			gLastSentCmdIndex -= 10; 

		while (gLastSentCmdIndex <= gCmdIndex) 
		{
			SvMsg m;
			FillMsg(m);
			usrcmd& c = commands[gLastSentCmdIndex & CMD_MASK];
			m.serverTime = commands[gLastSentCmdIndex & CMD_MASK].serverTime;
			m.cmd = c;

#if LOG_PACKETS
			std::cout << "packet send: "; Dump(m);
#endif

			gLastSentCmdIndex++; 

			ReverseTimeStamp(m);
			gPeer->Send(reinterpret_cast<char*>(&m), sizeof(SvMsg), HIGH_PRIORITY, UNRELIABLE_SEQUENCED, 0, RakNet::UNASSIGNED_RAKNET_GUID, true);
		}	

		RakNet::Packet *packet;
		for (packet=gPeer->Receive(); packet; gPeer->DeallocatePacket(packet), packet=gPeer->Receive())
		{
			unsigned char type = GetPacketIdentifier(packet);
			switch (type)
			{
				case ID_CL_MSG: 
					{
						ClMsg* m = reinterpret_cast<ClMsg*>(packet->data);
						assert(packet->length == sizeof(ClMsg));
						gPlayerAckState = m->state;

#if LOG_PACKETS
						std::cout << "ackState: "; Dump(*m);
#endif
						break;
					}
				case ID_SV_MSG: 
					{
						assert(0);
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
}

char locase (char c)
{
    if (c >= 'A' && c <= 'Z') return c - ('a'-'A');
    else return c;
}

static void commandRelease(int cmd)
{
	cmd = locase(cmd);
	switch (cmd)
	{
		case 'w':
			gMoveFlags &= ~eMoveFrwd; 
			break;
		case 's':
			gMoveFlags &= ~eMoveBck; 
			break;
		case 'a':
			gMoveFlags &= ~eMoveLeft; 
			break;
		case 'd':
			gMoveFlags &= ~eMoveRight; 
			break;
		case 32://space
			break;
		default:
			std::cout << cmd;
			break;
	}
}

static void command (int cmd)
{
	cmd = locase(cmd);
	switch (cmd)
	{
		case 'w':
			gMoveFlags |= eMoveFrwd; 
			break;
		case 's':
			gMoveFlags |= eMoveBck; 
			break;
		case 'a':
			gMoveFlags |= eMoveLeft; 
			break;
		case 'd':
			gMoveFlags |= eMoveRight; 
			break;
		case 32://space
			gMoveFlags |= eMoveJump; 
			break;
		default:
			std::cout << cmd;
			break;
	}
}

void mouseMove(int dx, int dy)
{
	//std::cout << "rot" << dx << " " << dy << "\n";
	const float speed = 2.0f;
	if (dx < 0)
		gViewRot[0] += speed;
	else if (dx > 0) 
		gViewRot[0] -= speed;

	if (dy < 0)
		gViewRot[1] += speed;
	else if (dy > 0) 
		gViewRot[1] -= speed;

	if (gViewRot[1] > 89.0)
		gViewRot[1] = 89.0;
	else if (gViewRot[1] < -89.0)
		gViewRot[1] = -89.0;
}

int main (int argc, char **argv)
{
	std::cout << "Client\n";

	gPeer = RakNet::RakPeerInterface::GetInstance();
	assert(gPeer);

	gPeer->SetOccasionalPing(true);

	RakNet::SocketDescriptor* p_SD = new RakNet::SocketDescriptor();
	gPeer->ApplyNetworkSimulator(kPacketLoss, kPacketExtraLagMS, 0);
	gPeer->Startup(1, p_SD, 1);

	gPeer->Connect(kHost, kServerPort, 0, 0);

  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.commandRelease = &commandRelease;
  fn.mouseMove = &mouseMove;
	
  fn.stop = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

  dInitODE2(0);
  //dRandSetSeed (time(0));
  dRandSetSeed(1);
  createTest();

  dWorldSetGravity(world,0,0,kGravityZ);

  ground = dCreatePlane (space,0,0,1,0);
  // run simulation
  //dsSimulationLoop (argc,argv,600, 50, 1024, 800,&fn);
  dsSimulationLoop(argc,argv,600, 50, 800, 600,&fn);


	gLastSentTime = GetCurrTime();

  dJointGroupDestroy(contactgroup);
  dWorldDestroy(world);
  dCloseODE();

	return 0;	
}
