/*
Bullet Continuous Collision Detection and Physics Library
Ragdoll Demo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#define CONSTRAINT_DEBUG_SIZE 0.2f


#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "RagdollDemo.h"

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h> 
#include <ctime>
#include <string>
#include <sstream> 

// Enrico: Shouldn't these three variables be real constants and not defines?

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif



struct collisionObject
{
	int id;
	bool hit;
	btRigidBody* body;

	//DEFAULT INIT!
	collisionObject(int i, btRigidBody* b): id(i), hit(false), body(b) {}
};


class RagDoll
{
	enum
	{
		BODYPART_PELVIS = 0,
		BODYPART_SPINE,
		BODYPART_HEAD,

		BODYPART_LEFT_UPPER_LEG,
		BODYPART_LEFT_LOWER_LEG,

		BODYPART_RIGHT_UPPER_LEG,
		BODYPART_RIGHT_LOWER_LEG,

		BODYPART_LEFT_UPPER_ARM,
		BODYPART_LEFT_LOWER_ARM,

		BODYPART_RIGHT_UPPER_ARM,
		BODYPART_RIGHT_LOWER_ARM,

		BODYPART_COUNT
	};

	enum
	{
		JOINT_PELVIS_SPINE = 0,
		JOINT_SPINE_HEAD,

		JOINT_LEFT_HIP,
		JOINT_LEFT_KNEE,

		JOINT_RIGHT_HIP,
		JOINT_RIGHT_KNEE,

		JOINT_LEFT_SHOULDER,
		JOINT_LEFT_ELBOW,

		JOINT_RIGHT_SHOULDER,
		JOINT_RIGHT_ELBOW,

		JOINT_COUNT
	};

	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];
	btTypedConstraint* m_joints[JOINT_COUNT];

	btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
	{
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			shape->calculateLocalInertia(mass,localInertia);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		m_ownerWorld->addRigidBody(body);

		return body;
	}

public:
	RagDoll (btDynamicsWorld* ownerWorld, const btVector3& positionOffset)
		: m_ownerWorld (ownerWorld)
	{
		/*
		// Setup the geometry
		m_shapes[BODYPART_PELVIS] = new btCapsuleShape(btScalar(0.15), btScalar(0.20));
		m_shapes[BODYPART_SPINE] = new btCapsuleShape(btScalar(0.15), btScalar(0.28));
		m_shapes[BODYPART_HEAD] = new btCapsuleShape(btScalar(0.10* 4), btScalar(0.05 * 4));
		m_shapes[BODYPART_LEFT_UPPER_LEG] = new btCapsuleShape(btScalar(0.07), btScalar(0.45));
		m_shapes[BODYPART_LEFT_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.37));
		m_shapes[BODYPART_RIGHT_UPPER_LEG] = new btCapsuleShape(btScalar(0.07), btScalar(0.45));
		m_shapes[BODYPART_RIGHT_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.37));
		m_shapes[BODYPART_LEFT_UPPER_ARM] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
		m_shapes[BODYPART_LEFT_LOWER_ARM] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));
		m_shapes[BODYPART_RIGHT_UPPER_ARM] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
		m_shapes[BODYPART_RIGHT_LOWER_ARM] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));

		// Setup all the rigid bodies
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset);

		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.), btScalar(0.)));
		m_bodies[BODYPART_PELVIS] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_PELVIS]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.2), btScalar(0.)));
		m_bodies[BODYPART_SPINE] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_SPINE]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.6), btScalar(0.)));
		m_bodies[BODYPART_HEAD] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_HEAD]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.18), btScalar(0.65), btScalar(0.)));
		m_bodies[BODYPART_LEFT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_UPPER_LEG]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.18), btScalar(0.2), btScalar(0.)));
		m_bodies[BODYPART_LEFT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_LOWER_LEG]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.18), btScalar(0.65), btScalar(0.)));
		m_bodies[BODYPART_RIGHT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_LEG]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.18), btScalar(0.2), btScalar(0.)));
		m_bodies[BODYPART_RIGHT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_LEG]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.35), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,M_PI_2);
		m_bodies[BODYPART_LEFT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_UPPER_ARM]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.7), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,M_PI_2);
		m_bodies[BODYPART_LEFT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_LOWER_ARM]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.35), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,-M_PI_2);
		m_bodies[BODYPART_RIGHT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_ARM]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.7), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,-M_PI_2);
		m_bodies[BODYPART_RIGHT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_ARM]);

		// Setup some damping on the m_bodies
		for (int i = 0; i < BODYPART_COUNT; ++i)
		{
			m_bodies[i]->setDamping(0.05, 0.85);
			m_bodies[i]->setDeactivationTime(0.8);
			m_bodies[i]->setSleepingThresholds(1.6, 2.5);
		}

		// Now setup the constraints
		btHingeConstraint* hingeC;
		btConeTwistConstraint* coneC;

		btTransform localA, localB;

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.15), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_SPINE], localA, localB);
		hingeC->setLimit(btScalar(-M_PI_4), btScalar(M_PI_2));
		m_joints[JOINT_PELVIS_SPINE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_SPINE], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI_2); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.30), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_HEAD], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, M_PI_2);
		m_joints[JOINT_SPINE_HEAD] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_SPINE_HEAD], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,-M_PI_4*5); localA.setOrigin(btVector3(btScalar(-0.18), btScalar(-0.10), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,-M_PI_4*5); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_LEFT_UPPER_LEG], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, 0);
		m_joints[JOINT_LEFT_HIP] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_HIP], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_LEFT_UPPER_LEG], *m_bodies[BODYPART_LEFT_LOWER_LEG], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_LEFT_KNEE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_KNEE], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI_4); localA.setOrigin(btVector3(btScalar(0.18), btScalar(-0.10), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_4); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_RIGHT_UPPER_LEG], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, 0);
		m_joints[JOINT_RIGHT_HIP] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_HIP], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_RIGHT_UPPER_LEG], *m_bodies[BODYPART_RIGHT_LOWER_LEG], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_RIGHT_KNEE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_KNEE], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI); localA.setOrigin(btVector3(btScalar(-0.2), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_LEFT_UPPER_ARM], localA, localB);
		coneC->setLimit(M_PI_2, M_PI_2, 0);
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_joints[JOINT_LEFT_SHOULDER] = coneC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_SHOULDER], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_LEFT_UPPER_ARM], *m_bodies[BODYPART_LEFT_LOWER_ARM], localA, localB);
//		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_LEFT_ELBOW] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_ELBOW], true);



		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,0); localA.setOrigin(btVector3(btScalar(0.2), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_RIGHT_UPPER_ARM], localA, localB);
		coneC->setLimit(M_PI_2, M_PI_2, 0);
		m_joints[JOINT_RIGHT_SHOULDER] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_SHOULDER], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_RIGHT_UPPER_ARM], *m_bodies[BODYPART_RIGHT_LOWER_ARM], localA, localB);
//		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_RIGHT_ELBOW] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_ELBOW], true);
		*/
	}

	virtual	~RagDoll ()
	{
		int i;

		// Remove all constraints
		for ( i = 0; i < JOINT_COUNT; ++i)
		{
			m_ownerWorld->removeConstraint(m_joints[i]);
			delete m_joints[i]; m_joints[i] = 0;
		}

		// Remove all bodies and shapes
		for ( i = 0; i < BODYPART_COUNT; ++i)
		{
			m_ownerWorld->removeRigidBody(m_bodies[i]);
			
			delete m_bodies[i]->getMotionState();

			delete m_bodies[i]; m_bodies[i] = 0;
			delete m_shapes[i]; m_shapes[i] = 0;
		}
	}
};

//----------------------------------------------------------------------------------------------------------
//--------------------------------NEW METHODS---------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------

//Assignment 5
void RagdollDemo::CreateBox(int index,double x, double y, double z,double length, 
double width, double height)
{ 
	geom[index] = new btBoxShape(btVector3(btScalar(length),btScalar(width),btScalar(height))); 
	btTransform offset; 
	offset.setIdentity(); 
	offset.setOrigin(btVector3(btScalar(x),btScalar(y),btScalar(z))); 
	body[index] = localCreateRigidBody(btScalar(1.0),offset,geom[index]); 
	body[index]->setCollisionFlags(body[index]->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	collisionID[currentIndex] = new collisionObject(currentIndex, body[index]);
	body[index]->setUserPointer(collisionID[currentIndex]);
	/*
	cout << "OBJECT ID: " << collisionID[currentIndex]->id;
	cout << " | POINTER ID: " << ((collisionObject*)body[index]->getUserPointer())->id;
	cout << " | ADDRESS " << collisionID[currentIndex];
	cout << " | FLAGS " << collisionID[currentIndex]->body->getCollisionFlags() << endl;
	*/
	currentIndex++;
} 

void RagdollDemo::CreateCylinder(int index,double x, double y, double z,double length, double radius, double eulerX, double eulerY, double eulerZ)
{ 
	geom[index] = new btCylinderShape(btVector3(btScalar(radius),btScalar(length),btScalar(0)));
	btTransform offset; 
	offset.setIdentity(); 
	offset.setOrigin(btVector3(btScalar(x),btScalar(y),btScalar(z)));

	btTransform transform;
	transform.setOrigin(btVector3(btScalar(0),btScalar(1),btScalar(0)));
	transform.getBasis().setEulerZYX(eulerX,eulerY,eulerZ); 

	body[index] = localCreateRigidBody(btScalar(1.0),offset*transform,geom[index]);
	body[index]->setCollisionFlags(body[index]->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	collisionID[currentIndex] = new collisionObject(currentIndex, body[index]);
	body[index]->setUserPointer(collisionID[currentIndex]);
	
	/*
	cout << "OBJECT ID: " << collisionID[currentIndex]->id;
	cout << " | POINTER ID: " << ((collisionObject*)body[index]->getUserPointer())->id;
	cout << " | ADDRESS " << collisionID[currentIndex];
	cout << " | FLAGS " << collisionID[currentIndex]->body->getCollisionFlags() << endl;
	*/
	currentIndex++;
} 

//ASSIGNMENT 8
static RagdollDemo* ragdolldemo;
bool callBackFunc(btManifoldPoint& cp, const btCollisionObjectWrapper* obj1, int id1, int index1, const btCollisionObjectWrapper* obj2, int id2, int index2)
{
	//CONSOL OUTPUT
	//cout << ((collisionObject*)obj1->getCollisionObject()->getUserPointer())->id << " - " << ((collisionObject*)obj2->getCollisionObject()->getUserPointer())->id << endl;

	//SAVE HIT INFORMATION TO STR
	((collisionObject*)obj1->getCollisionObject()->getUserPointer())->hit = true;
	((collisionObject*)obj2->getCollisionObject()->getUserPointer())->hit = true;

	//SAVE POSITION OF TOUCH
	ragdolldemo->touches[((collisionObject*)obj1->getCollisionObject()->getUserPointer())->id] = cp.m_positionWorldOnB;
	ragdolldemo->touches[((collisionObject*)obj2->getCollisionObject()->getUserPointer())->id] = cp.m_positionWorldOnB;
	return false;
}

double RagdollDemo::getRandomNumber()
{
	double randomNumber = (rand() % 100)/(double)100; 
	return randomNumber;
}

double RagdollDemo::getRandomAngle(int maxMove)
{
	double random = 0.0;
	if(rand()/2 % 2 == 0)
	{
		random = rand() % maxMove;
	}
	else
	{
		random = rand() % maxMove;
		random = random * -1;
	}
	return random;
}

void RagdollDemo::CreateHinge(
		int jointIndex,
		int bodyAIndex,
		int bodyBIndex,
		const btVector3& axisInA,
		const btVector3& axisInB,
		const btVector3& pivotInA,
		const btVector3& pivotInB)
{
	btHingeConstraint* tempHinge = new btHingeConstraint(*body[bodyAIndex], *body[bodyBIndex], pivotInA, pivotInB, axisInA, axisInB);

	//ARM -> FEET JOINS
	if(jointIndex == 2)// || jointIndex == 3 || jointIndex == 5 || jointIndex == 6)
	{
		tempHinge->setLimit(-M_PI_4-M_PI_2, M_PI_4-M_PI_2); //***********
	}
	else if(jointIndex == 3)
	{
		tempHinge->setLimit(((2*M_PI_2)-M_PI_4)+M_PI_2, ((3*M_PI_2)-M_PI_4)+M_PI_2); //***********
	}
	else if(jointIndex == 5)
	{
		tempHinge->setLimit(((2*M_PI_2)-M_PI_4)-M_PI_2, ((3*M_PI_2)-M_PI_4)-M_PI_2); //************
	}
	else if(jointIndex == 6)
	{
		tempHinge->setLimit(-M_PI_4+M_PI_2, M_PI_4+M_PI_2); //************
	}

	//BODY -> ARM JOINTS
	if(jointIndex == 0)
	{
		tempHinge->setLimit((3*M_PI_2)-M_PI_4, (3*M_PI_2)+M_PI_4); //***************
	}
	else if(jointIndex == 1)
	{
		tempHinge->setLimit((3*M_PI_2)-M_PI_4, (3*M_PI_2)+M_PI_4); //***************
	}
	else if(jointIndex == 4)
	{
		tempHinge->setLimit(-3*M_PI_4, -M_PI_4); //***************
	}
	else if(jointIndex == 7)
	{
		tempHinge->setLimit(-3*M_PI_4, -M_PI_4); //***************
	}

	joint[jointIndex] = tempHinge;
	m_dynamicsWorld->addConstraint(tempHinge, true);
}

void RagdollDemo::ActuateJoint(int jointIndex, double desiredAngle, double jointOffset, double timeStep)
{
	joint[jointIndex]->enableMotor(TRUE);
	joint[jointIndex]->setMaxMotorImpulse(2); //2
	joint[jointIndex]->setMotorTarget(desiredAngle, .15); //.125
}

//----------------------------------------------------------------------------------------------------------
//--------------------------------END NEW METHODS-----------------------------------------------------------
//----------------------------------------------------------------------------------------------------------


void RagdollDemo::initPhysics()
{
	//ASSIGNMENT 8
	ragdolldemo = this;

	//ASSIGNMENT 9
	srand(time(NULL));

	//ASSIGNMENT 10
	pause = false;

	//ragdolldemo->touches = new btVector3*
	
	// Setup the basic world

	//setTexturing(true);
	//setShadows(true);

	setCameraDistance(btScalar(5.));

	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);

	m_solver = new btSequentialImpulseConstraintSolver;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	//m_dynamicsWorld->getDispatchInfo().m_useConvexConservativeDistanceUtil = true;
	//m_dynamicsWorld->getDispatchInfo().m_convexConservativeDistanceThreshold = 0.01f;
	


	//ASSIGNMENT 10
	string line;
	ifstream myfile("../../RAGDOLL_DATA/weights.txt");	
	//cout << "NORMAL SYNAPSE" << endl;
	for(int i = 0; i < 4; i++)
	{
		for(int k = 0; k < 8; k++)
		{
			getline(myfile,line);
			stringstream convert(line);
			convert >> synapseWeights[i][k];
			//cout << synapseWeights[i][k] << " | ";
		}
		//cout << endl;
	}
	myfile.close();
	
	//GET RECURRENT SYNAPSE
	line = "";
	ifstream myfile3("../../RAGDOLL_DATA/recurrent.txt");	
	//cout << "RECURRNT SYNAPSE" << endl;
	for(int i = 0; i < 4; i++)
	{
		for(int k = 0; k < 8; k++)
		{
			getline(myfile3,line);
			stringstream convert(line);
			convert >> synapseRecurrent[i][k];
			//cout << synapseRecurrent[i][k] << " | ";
		}
		//cout << endl;
	}
	myfile.close();

	// GET MASK
	line = "";
	//cout << "PRINTING RECURRENT" << endl;
	ifstream myfile2("../../RAGDOLL_DATA/mask.txt");	
	bool foundOn = false;
	bool foundOff = false;
	for(int i = 0; i < 4; i++)
	{
		for(int k = 0; k < 8; k++)
		{
			getline(myfile2,line);
			stringstream convert(line);
			convert >> mask[i][k];
			//cout << mask[i][k] << " | ";
			if(mask[i][k] == 1)
			{
				foundOn = true;
			}
			else if(mask[i][k] == 0)
			{
				foundOff = true;
			}
		}
		//cout << endl;
	}
	myfile2.close();

	if(!foundOn && foundOff)
	{
		maskMode = 0;
	}
	else if(!foundOff && foundOn)
	{
		maskMode = 1;
	}
	else
	{
		maskMode = 2;
	}

	//cout << "INITIAL PEVIOUS TOUCH" << endl;
	//INITIAL PREVIOUS STEP = 0
	for(int i = 0; i < 4; i++)
	{
		previousTouch[i] = 0;
		//cout << previousTouch[i] << " | ";
	}
	//cout << endl;


	
	
	// Setup a big ground box
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-10,0));

		#define CREATE_GROUND_COLLISION_OBJECT 1
		#ifdef CREATE_GROUND_COLLISION_OBJECT
			//ASSIGNMENT 8
		/*
			btTransform offset; 
			offset.setIdentity(); 
			offset.setOrigin(btVector3(btScalar(0),btScalar(-10),btScalar(0))); 
			btRigidBody* fixedGround = localCreateRigidBody(btScalar(1.0),offset, groundShape); 
			fixedGround->setCollisionFlags(3);
			*/
			btCollisionObject* fixedGround = new btCollisionObject();
			fixedGround->setFriction(btScalar(10000.0));
			fixedGround->setCollisionShape(groundShape);
			fixedGround->setWorldTransform(groundTransform);
			m_dynamicsWorld->addCollisionObject(fixedGround);



			currentIndex = 0;
			/*
			fixedGround->setCollisionFlags(fixedGround->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
			fixedGround->setCenterOfMassTransform(offset);
			*/
			collisionID[currentIndex] = new collisionObject(currentIndex, (btRigidBody *)groundShape);
			fixedGround->setUserPointer(collisionID[currentIndex]);

			/*
			cout << "OBJECT ID: " << collisionID[currentIndex]->id;
			cout << " | POINTER ID: " << ((collisionObject*)fixedGround->getUserPointer())->id;
			cout << " | ADDRESS " << collisionID[currentIndex];
			cout << " | FLAGS " << collisionID[currentIndex]->body->getCollisionFlags() << endl;
			*/
			currentIndex++;		
		#else
				localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
		#endif //CREATE_GROUND_COLLISION_OBJECT
	}

	gContactAddedCallback = callBackFunc;

	//BODY ------------------------------------------------------------------
	//(index, x, y, z, length, width, height)
	RagdollDemo::CreateBox(0, 0.0, 1.8, 0.0, 1.0, 0.2, 1.0);//WORKING


	//FEET
	RagdollDemo::CreateCylinder(3, 3.2, 0, 0, 1, .2, 0, M_PI_2, 0); //WORKING -> NORMAL
	RagdollDemo::CreateCylinder(4, -3.2, 0, 0, 1, .2, 0, -M_PI_2, 0); //WORKING -> NORMAL 
	RagdollDemo::CreateCylinder(8, 0, 0, -3.2, 1, .2, 0, M_PI_2, 0); //WORKING -> ROTATED
	RagdollDemo::CreateCylinder(6, 0, 0, 3.2, 1, .2, 0, M_PI_2, 0); //WORKING -> NORMAL ->  
	//ARM
	RagdollDemo::CreateCylinder(5, 0.0, 0.8, 2.0, 1.0, 0.2, M_PI_2, 0.0, 0.0);//WORKING -> ROTATED
	RagdollDemo::CreateCylinder(7, 0.0, 0.8, -2.0, 1.0, 0.2, M_PI_2, 0.0, 0.0);//WORKIN G-> ROTATED
	RagdollDemo::CreateCylinder(1, 2.0, .8, 0.0, 1.0, 0.2, 0.0, 0.0, M_PI_2);//WORKING -> NORMAL
	RagdollDemo::CreateCylinder(2, -2.0, .8, 0.0, 1.0, 0.2, 0, M_PI_2, M_PI_2);//WORKING -> NORMAL

	//FEET HINGE
	RagdollDemo::CreateHinge(2,3,1, btVector3(0.0, 0.0, 1.0), btVector3(0.0, 0.0, 1.0),btVector3(0.0, 1.0, 0.0), btVector3(0, -1.0, 0.0));//WORKING
	RagdollDemo::CreateHinge(3,4,2, btVector3(1.0, 0.0, 0.0), btVector3(0.0, 0.0, 1.0),btVector3(0.0, 1.0, 0.0), btVector3(0, 1.0, 0.0)); //WORKIN
	RagdollDemo::CreateHinge(6,7,8, btVector3(1.0, 0.0, 0.0), btVector3(1.0, 0.0, 0.0),btVector3(0.0, -1.0, 0.0), btVector3(0, 1.0, 0.0));//WORKING -> ROTATED
	RagdollDemo::CreateHinge(5,5,6, btVector3(1.0, 0.0, 0.0), btVector3(1.0, 0.0, 0.0),btVector3(0.0, 1.0, 0.0), btVector3(0, 1.0, 0.0));//WORKING -> ROTATED //

	//ARM HINGE
	RagdollDemo::CreateHinge(4,0,5, btVector3(1.0, 0.0, 0.0), btVector3(1.0, 0.0, 0.0),btVector3(0.0, 0.0, 1.0), btVector3(0, -1.0, 0.0));//WORKING -> ROTATED
	RagdollDemo::CreateHinge(7,0,7, btVector3(1.0, 0.0, 0.0), btVector3(1.0, 0.0, 0.0),btVector3(0.0, 0.0, -1.0), btVector3(0, 1.0, 0.0));//WORKING -> ROTATED
	RagdollDemo::CreateHinge(0,0,1, btVector3(0.0, 0.0, 1.0), btVector3(0.0, 0.0, 1.0), btVector3(1.0, 0.0, 0.0), btVector3(0.0, 1.0, 0.0));//WORKING
 	RagdollDemo::CreateHinge(1,0,2, btVector3(0.0, 0.0, 1.0), btVector3(0.0, 0.0, 1.0),btVector3(-1.0, 0.0, 0.0), btVector3(0.0, -1.0, 0.0));//WORKING
	
	//clientResetScene();
}

void RagdollDemo::spawnRagdoll(const btVector3& startOffset)
{
	RagDoll* ragDoll = new RagDoll (m_dynamicsWorld, startOffset);
	m_ragdolls.push_back(ragDoll);
}	

void RagdollDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;

	glFlush();
	glutSwapBuffers();

	/*

	THIS IS WHERE THE OLD CODE WAS......

	*/
}

void RagdollDemo::saveAndClose()
{		
	//SAVE FITNESS
	btVector3 centerOfMass = body[0]->getCenterOfMassPosition();
	//cout << "Z POSITION OF CENTER OF MASS: " << centerOfMass.getZ() << endl;

	//SAVING RESTULTS
	ofstream myfile;
	myfile.open ("../../RAGDOLL_DATA/fit.txt");
	myfile << centerOfMass.getZ();
	myfile.close();

	
	ofstream myfile2;
	//cout << "++++++++++++++++++++++ MASK MODE: " << maskMode << endl;
	/*
	if(maskMode == 0)
	{
		//OFF
		myfile2.open ("../../RAGDOLL_DATA/hitInfoOff.txt");
	}
	else if(maskMode == 1)
	{
		myfile2.open ("../../RAGDOLL_DATA/hitInfoOn.txt");
	}
	else if(maskMode == 2)
	{
		myfile2.open ("../../RAGDOLL_DATA/hitInfoEvolved.txt");
	}
	*/
	myfile2.open ("../../RAGDOLL_DATA/hitInfo.txt");
	for(int i = 0; i < 100; i++)
	{
		for(int j = 0; j < 8; j++)
		{
			myfile2 << hitInfo[j][i] << "\n";
			//cout << hitInfo[j][i] << "|";
		}
		//cout << endl;
	}
	myfile2.close();
	

	//EXIT
	exit(0);
}

void RagdollDemo::calcMotorCommands()
{
	//NORMAL RUNTIME LOOP
	
	//double randomMultiplyer = M_PI_4/1.5;
	double randomMultiplyer = M_PI_4/1.5;
	double randomNum = 0.0;
	double temp;
	for (int j=0; j<4; j++)
	{
		int id = j + 2;
		//USING CURRENT TOUCH DATA
		randomNum = randomNum + (collisionID[id]->hit * synapseWeights[j][0]);
		//USING PREVIOUS TOUCH DATA
		randomNum = randomNum + (previousTouch[j] * synapseRecurrent[j][0]) * mask[j][0];
		//STORE PREVIOUS TOUCH DATA
		previousTouch[j] = collisionID[id]->hit;
	}
					
	randomNum = tanh(randomNum);
	//cout << (randomNum * randomMultiplyer) << " | ";
	ActuateJoint(0, (-M_PI_2) + (randomNum * randomMultiplyer), 0, 0); //WORKING CHECK -> ARM JOINT ***************
				
	randomNum = 0.0;
	for (int j=0; j<4; j++)
	{
		int id = j + 2;
		//USING CURRENT TOUCH DATA
		randomNum = randomNum + (collisionID[id]->hit * synapseWeights[j][1]);
		//USING PREVIOUS TOUCH DATA
		randomNum = randomNum + (previousTouch[j] * synapseRecurrent[j][1]) * mask[j][1];
		//STORE PREVIOUS TOUCH DATA
		previousTouch[j] = collisionID[id]->hit;
	}
	randomNum = tanh(randomNum);
	//cout << (randomNum * randomMultiplyer) << " | ";
	ActuateJoint(1, (-M_PI_2) + (randomNum *  randomMultiplyer), 0, 0); //WORKING CHECK -> ARM JOINT ************** [][][]

	//ROTATE
	randomNum = 0.0;
	for (int j=0; j<4; j++)
	{
		int id = j + 2;
		//USING CURRENT TOUCH DATA
		randomNum = randomNum + (collisionID[id]->hit * synapseWeights[j][4]);
		//USING PREVIOUS TOUCH DATA
		randomNum = randomNum + (previousTouch[j] * synapseRecurrent[j][4]) * mask[j][4];
		//STORE PREVIOUS TOUCH DATA
		previousTouch[j] = collisionID[id]->hit;
	}
			
	randomNum = tanh(randomNum);
	//cout << (randomNum * randomMultiplyer) << " | ";
	ActuateJoint(4, (-M_PI_2) + (randomNum *  randomMultiplyer), 0, 0); //WORKING CHECK -> ARM JOINT ************ [][][]
				
	randomNum = 0.0;
	for (int j=0; j<4; j++)
	{
		int id = j + 2;
		//USING CURRENT TOUCH DATA
		randomNum = randomNum + (collisionID[id]->hit * synapseWeights[j][7]);
		//USING PREVIOUS TOUCH DATA
		randomNum = randomNum + (previousTouch[j] * synapseRecurrent[j][7]) * mask[j][7];
		//STORE PREVIOUS TOUCH DATA
		previousTouch[j] = collisionID[id]->hit;
	}
	randomNum = tanh(randomNum);
	//cout << (randomNum * randomMultiplyer) << " | ";
	ActuateJoint(7, (-M_PI_2) + (randomNum *  randomMultiplyer), 0, 0); //WORKING CHECK -> ARM JOINT **************

	//ARMS -> FEET
	//NORMAL
	randomNum = 0.0;
	for (int j=0; j<4; j++)
	{
		int id = j + 2;
		//USING CURRENT TOUCH DATA
		randomNum = randomNum + (collisionID[id]->hit * synapseWeights[j][3]);
		//USING PREVIOUS TOUCH DATA
		randomNum = randomNum + (previousTouch[j] * synapseRecurrent[j][3]) * mask[j][3];
		//STORE PREVIOUS TOUCH DATA
		previousTouch[j] = collisionID[id]->hit;
	}
	randomNum = tanh(randomNum);
	//cout << (randomNum * randomMultiplyer) << " | ";
	ActuateJoint(3, (-M_PI_2) + (randomNum *  randomMultiplyer), 0, 0);//WORKING CHECK -> LEG JOINT ***********************
	randomNum = 0.0;
	for (int j=0; j<4; j++)
	{
		int id = j + 2;
		//USING CURRENT TOUCH DATA
		randomNum = randomNum + (collisionID[id]->hit * synapseWeights[j][2]);
		//USING PREVIOUS TOUCH DATA
		randomNum = randomNum + (previousTouch[j] * synapseRecurrent[j][2]) * mask[j][2];
		//STORE PREVIOUS TOUCH DATA
		previousTouch[j] = collisionID[id]->hit;
	}
	randomNum = tanh(randomNum);
	//cout << (randomNum * randomMultiplyer) << " | ";
	ActuateJoint(2, (-M_PI_2) + (randomNum *  randomMultiplyer), 0, 0); //WORKING CHECK -> LEG JOINT ***************
				
	//ROTATED
	randomNum = 0.0;
	for (int j=0; j<4; j++)
	{
		int id = j + 2;
		//USING CURRENT TOUCH DATA
		randomNum = randomNum + (collisionID[id]->hit * synapseWeights[j][6]);
		//USING PREVIOUS TOUCH DATA
		randomNum = randomNum + (previousTouch[j] * synapseRecurrent[j][6]) * mask[j][6];
		//STORE PREVIOUS TOUCH DATA
		previousTouch[j] = collisionID[id]->hit;
	}
	randomNum = tanh(randomNum);
	//cout << (randomNum * randomMultiplyer) << " | ";
	ActuateJoint(6, (M_PI_2) + (randomNum *  randomMultiplyer), 0, 0); //WORKING CHECK -> LEG JOINT*****************
				
	randomNum = 0.0;
	for (int j=0; j<4; j++)
	{
		int id = j + 2;
		//USING CURRENT TOUCH DATA
		randomNum = randomNum + (collisionID[id]->hit * synapseWeights[j][5]);
		//USING PREVIOUS TOUCH DATA
		randomNum = randomNum + (previousTouch[j] * synapseRecurrent[j][5]) * mask[j][5];
		//STORE PREVIOUS TOUCH DATA
		previousTouch[j] = collisionID[id]->hit;
	}
	randomNum = tanh(randomNum);
	//cout << (randomNum * randomMultiplyer) << " | ";
	ActuateJoint(5, (M_PI_2) + (randomNum *  randomMultiplyer), 0, 0); // RORATION OFFF -> LEG JOINT ************

				


	//OLD TIME STEP INCREMENTER PLACE!!!
	oneStep = FALSE;	

	totalTimeStepCounter++;
		
}

void RagdollDemo::showHitInfo()
{
	for(int i = 2; i < 10; i++)
	{
		if(collisionID[i]->hit)
		{
			//cout << "##";
		}
		else
		{
			//cout << "  ";
		}		
		collisionID[i]->hit = false;
		ragdolldemo->touches[i] = btVector3(btScalar(200),btScalar(0),btScalar(200));
	}
	//cout << endl;
}

void RagdollDemo::saveHitInfo(int counter)
{
	//cout << "------" << counter << endl;
	for(int i = 2; i < 10; i++)
	{
		if(collisionID[i]->hit)
		{
			hitInfo[i-2][counter] = 1;
		}
		else
		{
			hitInfo[i-2][counter] = 0;
		}
		//cout << hitInfo[i-2][counter] << " | ";
		collisionID[i]->hit = false;
	}
	//cout << endl;
}

void RagdollDemo::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		//m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

void RagdollDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'e':
		{
		btVector3 startOffset(0,2,0);
		spawnRagdoll(startOffset);
		break;
		}
	default:
		DemoApplication::keyboardCallback(key, x, y);
	case 'p':
		{
			if(pause)
				pause = FALSE;
			else
				pause = TRUE;
		}
	case 'o':
		{
			if(oneStep)
			{
				oneStep = FALSE;
			}
			else
			{
				oneStep = TRUE;	
			}
		}
	case 'z':
		{
			//cout << "SIZE: " << sizeof(collisionID)/sizeof(collisionID[0]) << endl;
			
			for(int i = 0; i < sizeof(collisionID)/sizeof(collisionID[0]); i++)
			{
				/*
				cout << i << ": " << collisionID[i]->id;
				cout << " | ADDRESS: " << collisionID[i];
				cout << " | HIT: " << collisionID[i]->hit;
				cout << " | Colliosion Flags: " << collisionID[i]->body->getCollisionFlags() << endl;
				*/
			}
			
		}		
	}

	
}



void	RagdollDemo::exitPhysics()
{
	DeleteObject(0);
	/*
	int i;

	for (i=0;i<m_ragdolls.size();i++)
	{
		RagDoll* doll = m_ragdolls[i];
		delete doll;
	}

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;
	*/
	
}





