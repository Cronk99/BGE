#include "assignment.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"


using namespace BGE;

assignment::assignment(void)
{
	physicsFactory = NULL;
	dynamicsWorld = NULL;
	broadphase = NULL;
	dispatcher = NULL;
	solver = NULL;
	fullscreen = false;
}


assignment::~assignment(void)
{
}

shared_ptr<PhysicsController> cyl_2;
std::shared_ptr<GameComponent> station2;

bool assignment::Initialise() 
{
	riftEnabled = false;
	// Set up the collision configuration and dispatcher
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
 
    // The world.
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	broadphase = new btAxisSweep3(worldMin,worldMax);
	solver = new btSequentialImpulseConstraintSolver();
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0,0,0));


	physicsFactory = make_shared<PhysicsFactory>(dynamicsWorld);

	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();
	

	//assignment
	//**************************************************************************************
	snitch = make_shared<GameComponent>();
	snitch->Attach(Content::LoadModel("python", glm::rotate(glm::mat4(1), 180.0f, glm::vec3(0,1,0))));
	snitch->Attach(make_shared<VectorDrawer>(glm::vec3(5,5,5)));
	snitch->position = glm::vec3(-25,0,-20);
	Attach(snitch);

	//create the first chaser
	ship2 = make_shared<GameComponent>();
	ship2->Attach(Content::LoadModel("cobramk3", glm::rotate(glm::mat4(1), 180.0f, glm::vec3(0,1,0))));
	ship2->Attach(make_shared<VectorDrawer>(glm::vec3(5,5,5)));
	ship2->position = glm::vec3(-45,0,10);
	Attach(ship2);

	//create the second chaser
	ship3 = make_shared<GameComponent>();
	ship3->Attach(Content::LoadModel("cobramk3", glm::rotate(glm::mat4(1), 180.0f, glm::vec3(0,1,0))));
	ship3->Attach(make_shared<VectorDrawer>(glm::vec3(5,5,5)));
	ship3->position = glm::vec3(-15,0,10);
	Attach(ship3);

	//Create the stadium
	physicsFactory->CreateWall(glm::vec3(-50,0,20), 10, 1);
	physicsFactory->CreateWall(glm::vec3(-50,0,-50), 10, 1);
	shared_ptr<PhysicsController> stadiumWall = physicsFactory->CreateBox(5,5,75, glm::vec3(-57,0,-15), glm::quat()); 
	shared_ptr<PhysicsController> stadiumWall2 = physicsFactory->CreateBox(5,5,75, glm::vec3(20,0,-15), glm::quat()); 

	//magic for following the snitch
	magic = make_shared<FountainEffect>(500);
	magic->position = snitch->position;
	magic->diffuse = glm::vec3(1,1, 0);
	Attach(magic);
	
	
	//create a selection of arms 
	for(int j = 3; j < 8; j ++) //creates a series of ever increasing in size hands
	{
		float z = j*10;
		physicsFactory->CreateArm(j,glm::vec3(60,80,z));
	}

	//create jenga tower

		physicsFactory->CreateJenga(15, 20, glm::vec3 (0,3,-100));
	

	if (!Game::Initialise()) {
		return false;
	}

	camera->GetController()->position = glm::vec3(-40,10, 30);
	
	return true;
}

void BGE::assignment::Update(float timeDelta)
{
	
	//snitch moving
	snitch->position += snitch->look * (speed * 3) * timeDelta;
	magic->position = snitch->position;
	if(snitch->position.z < -40)
	{
		int r = rand() % 180;
		snitch->Yaw(r);
	}
	if(snitch->position.z > 15)
	{
		int r = rand() % 180;
		snitch->Yaw(r);
	}
	if(snitch->position.x > 12)
	{
		int r = rand() % 180;
		snitch->Yaw(r);
	}
	if(snitch->position.x < -48)
	{
		int r = rand() % 180;
		snitch->Yaw(r);
	}

	if (keyState[SDL_SCANCODE_UP])
	{
		camera->GetController()->position = snitch->position;
		camera->GetController()->look = snitch->look;
	}
	if (keyState[SDL_SCANCODE_LEFT])
	{
		camera->GetController()->position = ship2->position;
		camera->GetController()->look = ship2->look;
	}
	if (keyState[SDL_SCANCODE_RIGHT])
	{
		camera->GetController()->position = ship3->position;
		camera->GetController()->look = ship3->look;
	}
	if (keyState[SDL_SCANCODE_DOWN])
	{
		camera->GetController()->position = glm::vec3(-40,10, 30);
	}



	//ship 2 chases snitch
	ship2->position += ship2->look * (speed * 0.5f) * timeDelta;
	glm::vec3 seesnitch = snitch->position - ship2->position;
	seesnitch = glm::normalize(seesnitch);
	glm::vec3 axis = glm::cross(GameComponent::basisLook, seesnitch);
	axis = glm::normalize(axis);
	float theta = glm::acos(glm::dot(seesnitch, GameComponent::basisLook));
	ship2->orientation = glm::angleAxis(glm::degrees(theta), axis);
	/*if(glm::length(seesnitch) < 1)
	{
		snitch->position = glm::vec3(-25,0,-20);

		ship2->position = glm::vec3(-45,0,10);
	}*/
	
	
	//ship3 chases snitch
	ship3->position += ship3->look * (speed * 0.5f) * timeDelta;
	glm::vec3 seesnitch3 = snitch->position - ship3->position;
	seesnitch3 = glm::normalize(seesnitch3);
	glm::vec3 axis3 = glm::cross(GameComponent::basisLook, seesnitch3);
	axis = glm::normalize(axis3);
	float theta3 = glm::acos(glm::dot(seesnitch3, GameComponent::basisLook));
	ship3->orientation = glm::angleAxis(glm::degrees(theta3), axis3);
	/*if(glm::length(seesnitch) < 1)
	{
		snitch->position = glm::vec3(-25,0,-20);

		ship3->position = glm::vec3(-15,0,10);
	}*/
	


	//*********************************************************************


	dynamicsWorld->stepSimulation(timeDelta,100);
	Game::Update(timeDelta);
}

void BGE::assignment::Cleanup()
{
	Game::Cleanup();
}



