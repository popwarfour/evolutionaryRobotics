/*
Bullet Continuous Collision Detection and Physics Library
RagdollDemo
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

#ifndef RAGDOLLDEMO_H
#define RAGDOLLDEMO_H

#include <iostream>
#include <fstream>

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "GLDebugDrawer.h"

using namespace std;


class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;
class btHingeConstraint;

class RagdollDemo : public GlutDemoApplication
{
	/*
	btAlignedObjectArray<class RagDoll*> m_ragdolls;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;
	*/
	btAlignedObjectArray<class RagDoll*> m_ragdolls;
	//btAlignedObjectArray<class btCollisionShape*> m_ragdolls;
	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
	btBroadphaseInterface* m_broadphase;
	btCollisionDispatcher* m_dispatcher;
	btConstraintSolver* m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;

	//Assignment 5
	btRigidBody* body[9]; // one main body, 4x2 leg segments
	btCollisionShape* geom[9];
	bool pause;

	//Assignment 6
	btHingeConstraint* joint[8];
	bool oneStep;

	//ASSIGNMENT 8
	int currentIndex;
	(struct collisionObject*)collisionID[9];
	//(btVector3) touchPoints[10];
	
	
	//ASSIGNMENT 9
	double synapseWeights[4][8];

	//ASSIGNMENT FINAL PROJECT
	double previousTouch[4];
	double synapseRecurrent[4][8];
	int mask[4][8];
	bool recurrentOn;

	
public:
	btVector3 touches[10];
	

	//Assignment 5
	void RagdollDemo::CreateBox(int index,double x, double y, double z,double length, double width, double height);
	void RagdollDemo::CreateCylinder(int index,double x, double y, double z,double radius, double length, double eulerX, double eulerY, double eulerZ);


	//Assignmnet 6
	//btVector3 RagdollDemo::PointWorldToLocal(int bodyIndex, btVector3& point);
	//void RagdollDemo::CreateHinge(int jointIndex, int index1, int index2, btVector3& fulcumA, btVector3& fulcumB, btVector3& rotAxisA, btVector3& rotAxisB);
	void RagdollDemo::CreateHinge(
		int jointIndex,
		int bodyAIndex,
		int bodyBIndex,
		const btVector3& axisInA,
		const btVector3& axisInB,
		const btVector3& pivotInA,
		const btVector3& pivotInB);
	
	//Assignmnet 7
	void RagdollDemo::ActuateJoint(int jointIndex, double desiredAngle, double jointOffset, double timeStep);

	//Assignment 8
	double RagdollDemo::getRandomAngle(int maxMove);

	//Assignment 9
	double RagdollDemo::getRandomNumber();


	void initPhysics();

	void exitPhysics();

	virtual ~RagdollDemo()
	{
		exitPhysics();
	}

	void spawnRagdoll(const btVector3& startOffset);

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	//ASSIGNMENT 8
	virtual void renderme()
	{
		extern GLDebugDrawer gDebugDrawer;
		// Call the parent method.
		GlutDemoApplication::renderme();
		// Make a circle with a 0.9 radius at (0,0,0)
		// with RGB color (1,0,0).
		for(int i = 0; i < 10; i++)
		{
			gDebugDrawer.drawSphere(touches[i], 0.2, btVector3(1., 0., 0.));
		}
	}

	static DemoApplication* Create()
	{
		RagdollDemo* demo = new RagdollDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
	
};


#endif
