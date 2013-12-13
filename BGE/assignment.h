#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>
#include "FountainEffect.h"


namespace BGE
{
	class assignment :

	public Game
	{
	private:
		btBroadphaseInterface* broadphase;
 
		// Set up the collision configuration and dispatcher
		btDefaultCollisionConfiguration * collisionConfiguration;
		btCollisionDispatcher * dispatcher;
 
		// The actual physics solver
		btSequentialImpulseConstraintSolver * solver;

	public:
		assignment(void);
		~assignment(void);
		bool Initialise();
		void Update(float timeDelta);
		void Cleanup();
		void CreateWall();
		

		//assignment stuff
		//*************************************************
		shared_ptr<FountainEffect> magic;
		float swingTheta;

		shared_ptr<GameComponent> snitch;
		shared_ptr<GameComponent> ship2;
		shared_ptr<GameComponent> ship3;

		shared_ptr<PhysicsController> stadiumWall;
		shared_ptr<PhysicsController>stadiumWall2;

		//*************************************************


		// The world.
		std::shared_ptr<PhysicsFactory> physicsFactory;
		btDiscreteDynamicsWorld * dynamicsWorld;
	};
}