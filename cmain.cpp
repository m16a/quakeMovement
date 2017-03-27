#include <iostream>

#include "shared.h"


#include <time.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <cassert>

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

#define NUM 10			// number of bodies
#define NUMJ 9			// number of joints
#define SIDE (0.1)		// side length of a box
#define MASS (1.0)		// mass of a box
#define RADIUS (0.1732f)	// sphere radius
#define DENSITY (5.0)		// density of all objects

RakNet::RakPeerInterface* gPeer = 0;

const float kStepSize = 0.01f;
float gLastSentTime = 0; 

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

static int num = 4;


static float gViewRot[3] = {0.0f, 0.0f, 0.0f};
static bool gFlying = true;

int gCmdIndex = 0;
#define MAX_COMMANDS 64
#define CMD_MASK (MAX_COMMANDS -1)
usrcmd commands[MAX_COMMANDS];

enum 
{
	eMoveFrwd = 1,
	eMoveBck = 1 << 1,
	eMoveLeft = 1 << 2,
	eMoveRight = 1 << 3
};

static int gMoveFlags = 0;

void drawGeom(dGeomID g, const dReal *pos, const dReal *R, int show_aabb)
{
    int i;
	
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

    if (show_aabb) {
        // draw the bounding box for this geom
        dReal aabb[6];
        dGeomGetAABB(g,aabb);
        dVector3 bbpos;
        for (i=0; i<3; i++)
            bbpos[i] = 0.5*(aabb[i*2] + aabb[i*2+1]);
        dVector3 bbsides;
        for (i=0; i<3; i++)
            bbsides[i] = aabb[i*2+1] - aabb[i*2];
        dMatrix3 RI;
        dRSetIdentity (RI);
        dsSetColorAlpha(1,0,0,0.5);
        dsDrawBox(bbpos,RI,bbsides);
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

#if 1
	//test box
	obj[0].body = dBodyCreate(world);
	dMassSetBoxTotal(&m, 1, 10000000, 10000000, 10000000);//disable rotation
	obj[0].geom = dCreateBox(space, SIDE, SIDE, SIDE);
  dBodySetPosition(obj[0].body, 0, 0, 0.2);
	dBodySetMass(obj[0].body, &m);

	dVector3 impF;
	//dWorldImpulseToForce(world, kStepSize, 0.01, 0, 0, impF);
	//dBodyAddForceAtPos(obj[0].body, impF[0], impF[1], impF[2], -0.05, 0.05, 0.5);

	//dBodySetLinearVel(obj[0].body, -0.3, 0, 0.3);
	//dBodySetAngularVel(obj[0].body, 0, 0, 2);
	dGeomSetBody(obj[0].geom, obj[0].body);

	obj[1].geom = dCreateRay(space, SIDE / 2.0f + 0.01);
	dGeomRaySet(obj[1].geom, 0,0,0.2, 0,0,-SIDE/2.0f - 0.001);

	//ground contact check ray

#endif


	//test wall
	obj[2].geom = dCreateBox(space, 0.1, 1, 1);
  dGeomSetPosition(obj[2].geom, -1, 0, 1);

	//test wall
	obj[3].geom = dCreateBox(space, 0.1,1, 1);
  dGeomSetPosition(obj[3].geom, 1, 0, 1);


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

static void simLoop (int pause)
{
  double dt = dsElapsedTime();
	gFlying = true;

  int nrofsteps = (int) ceilf(dt/kStepSize);
	
  for (int i=0; i<nrofsteps && !pause; i++)
  {
		static float simTime = 0.0f;
		static int frameNum = 0;
    dSpaceCollide (space,0,&nearCallback);
    dWorldQuickStep (world, kStepSize);
    dJointGroupEmpty (contactgroup);
		simTime += kStepSize;	
		const dReal* pos = dBodyGetPosition(obj[0].body);
		const dReal* rot = dBodyGetQuaternion(obj[0].body);
		const dReal* w = dBodyGetAngularVel(obj[0].body);
		const dReal* v = dBodyGetLinearVel(obj[0].body);
	
		//fprintf(stdout, "[%d]sT=%.3f pos(%.3f %.3f %.3f) vel:(%.3f %.3f %.3f)  rot(%.3f %.3f %.3f %.3f) w(%.3f %.3f %.3f)\n",frameNum, simTime, pos[0], pos[1], pos[2], v[0], v[1], v[2], rot[0], rot[1], rot[2], rot[3], w[0], w[1], w[2]);
		frameNum++;
	dMass tmp;
	dBodyGetMass (obj[0].body, &tmp);
	dReal* J = tmp.I;

	//fprintf(stdout, "finite rotation:%d", dBodyGetFiniteRotationMode(obj[0].body));
	//fprintf(stdout, "J(%.5f %.5f %.5f %.3f)(%.5f %.5f %.5f %.3f)(%.5f %.5f %.5f %.3f)\n", J[0],J[1],J[2],J[3],J[4],J[5],J[6],J[7],J[8],J[9],J[10],J[11]);
  }

	// remove all contact joints
	dJointGroupEmpty(contactgroup);

	dsSetTexture(DS_WOOD);
	for (int i=0; i<num; i++) {
					if (0) {
							dsSetColor(0,0.7,1);
					} else if (obj[i].body && !dBodyIsEnabled(obj[i].body)) {
							dsSetColor(1,0.8,0);
					} else {
							dsSetColor(1,1,0);
					}
					drawGeom(obj[i].geom,0,0,0);
	}

	const dReal* pos = dBodyGetPosition(obj[0].body);
  //static float xyz[3] = {0.0f, -2.0f, 1.0f};
	//movement

	int vec[2] = {0,0};
	if (gMoveFlags & eMoveFrwd)
		vec[0] += 1;
	if (gMoveFlags & eMoveBck)
		vec[0] -= 1;

	if (gMoveFlags & eMoveRight)
		vec[1] += 1;
	if (gMoveFlags & eMoveLeft)
		vec[1] -= 1;

	float dir2d[2] = {cos(gViewRot[0] / 180.0f * M_PI), sin(gViewRot[0] / 180.0f * M_PI)};
	//std::cout << "dir2d:"<< dir2d[0] << "-" << dir2d[1] << "\n";
	if (vec[0] || vec[1])
	{
		//std::cout << "move:"<< vec[0] << "-" << vec[1] << "\n";
		// dir2d.cross(0,0,1)
		float res[2] = {vec[0] * dir2d[0] + vec[1] * dir2d[1], vec[0] * dir2d[1] - vec[1] * dir2d[0]};
		
		float res_norm = sqrt(res[0]*res[0] + res[1]*res[1]);

		res[0] /= res_norm;
		res[1] /= res_norm;

		float speed = 1;
		if (!gFlying)
		{
			const dReal* v = dBodyGetLinearVel(obj[0].body);
			//dBodySetLinearVel(obj[0].body, speed * res[0], speed * res[1], v[2]);

			usrcmd& c = commands[(++gCmdIndex) & CMD_MASK];
			c.serverTime = RakNet::GetTime();
			c.forward = static_cast<signed char>(speed * res[0] * 100.0f);
			c.right = static_cast<signed char>(speed * res[1] * 100.0f);
		}
	}

	float offset = 0.0;
  float xyz[3] = {pos[0] - offset*dir2d[0], pos[1] - offset*dir2d[1], pos[2]+0.1};
  dsSetViewpoint(xyz,gViewRot);

	//std::cout << "f:" << gFlying << "\n";
	dGeomRaySet(obj[1].geom, pos[0], pos[1], pos[2], 0,0,-SIDE/2.0f - 0.001);

	// NETWORK STAF

	{
		const float currT = GetCurrTime();
		if (currT < gLastSentTime + 0.05)
			return;

		gLastSentTime = currT;
		Msg m;
		FillMsg(m);
		m.forward = commands[gCmdIndex & CMD_MASK].forward;
		m.right = commands[gCmdIndex & CMD_MASK].right;
		m.timeStamp = commands[gCmdIndex & CMD_MASK].serverTime;

		//Dump(m);

#if USE_BIT_STREAM 
		RakNet::BitStream myBitStream;
		myBitStream.Write(m.useTimeStamp);
		myBitStream.Write(m.timeStamp);
		myBitStream.Write(m.typeId);
		myBitStream.Write(m.x);
		myBitStream.Write(m.y);
		myBitStream.Write(m.z);
		gPeer->Send(&myBitStream, HIGH_PRIORITY, RELIABLE, 0, RakNet::UNASSIGNED_RAKNET_GUID, true);
#else
		ReverseTimeStamp(m);
		gPeer->Send(reinterpret_cast<char*>(&m), sizeof(Msg), HIGH_PRIORITY, RELIABLE, 0, RakNet::UNASSIGNED_RAKNET_GUID, true);
#endif

		//std::cout << "sent t:" << currT << "\n";
		

		RakNet::Packet *packet;
		for (packet=gPeer->Receive(); packet; gPeer->DeallocatePacket(packet), packet=gPeer->Receive())
		{
			//std::cout << "reading\t";
			unsigned char type = GetPacketIdentifier(packet);
			switch (type)
			{
				case ID_MY_MSG: 
					{
						Msg* m = reinterpret_cast<Msg*>(packet->data);
						assert(packet->length == sizeof(Msg));

						//Dump(*m);
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
			if (!gFlying)
			{
				const dReal* v = dBodyGetLinearVel(obj[0].body);
				dBodySetLinearVel(obj[0].body, v[0], v[1], v[2] + 1);
			}
			break;
		default:
			std::cout << cmd;
			break;
	}
}

void mouseMove(int dx, int dy)
{
	//std::cout << "rot" << dx << " " << dy << "\n";
	const float speed = 0.8f;
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
	gPeer->Startup(1, p_SD, 1);

	gPeer->Connect(kHost, kServerPort, 0, 0);

  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.commandRelease = &commandRelease;
  fn.mouseMove= &mouseMove;
	
  fn.stop = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

  dInitODE2(0);
  dRandSetSeed (time(0));
  createTest();

  dWorldSetGravity (world,0,0,-0.8);

  ground = dCreatePlane (space,0,0,1,0);
  // run simulation
  dsSimulationLoop (argc,argv,800,600,&fn);


	gLastSentTime = GetCurrTime();

  dJointGroupDestroy(contactgroup);
  dWorldDestroy (world);
  dCloseODE();

	return 0;	
}
