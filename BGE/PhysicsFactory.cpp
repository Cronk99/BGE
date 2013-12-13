#include "PhysicsFactory.h"
#include "Game.h"
#include "Sphere.h"
#include "Box.h"
#include "Cylinder.h"
#include "Ground.h"
#include "Content.h"
#include "PhysicsCamera.h"
#include "Model.h"
#include "dirent.h"
#include "Utils.h"
using namespace BGE;

PhysicsFactory::PhysicsFactory(btDiscreteDynamicsWorld * dynamicsWorld)
{
	this->dynamicsWorld = dynamicsWorld;
}


PhysicsFactory::~PhysicsFactory(void)
{
}

void PhysicsFactory::CreateWall(glm::vec3 startAt, float width, float height, float blockWidth, float blockHeight, float blockDepth)
{
	float z = startAt.z;
	float gap = 1;

	for (int w = 0 ; w < width ; w ++)
	{
		for (int h = 0 ; h < height ; h ++)	
		{
			float x = startAt.x + ((blockWidth + 2) * w);
			float y = ((blockHeight + gap) / 2.0f) + ((blockHeight + gap) * h);
			CreateBox(blockWidth, blockHeight, blockDepth, glm::vec3(x, y, z), glm::quat());
		}
	}
}



//Create a Jenga tower
void PhysicsFactory::CreateJenga(float blocklength, float towerHeight, glm::vec3 pos)
{
	float gap = blocklength /2; //for the gap inbetween the blocks

	float otherBlocks = blocklength * 0.2; //sets the width and depth for th blocks

	for(int i =0; i < towerHeight; i++)
	{

		float z_change = pos.z; //hold the z  position in temporary variable
		float x_change = pos.x; //hold the x position in temporary variable

		//create every even layer
		if(i % 2 == 0)
		{
			for(int j = 1; j< 4;j++)
			{    //this makes the even layer of boxes that stretch out across the x-axis and places 3 across
				CreateBox(blocklength,otherBlocks,otherBlocks, glm::vec3(pos.x, pos.y, z_change + (gap * (j*0.7))),glm::quat());

			}
		}
		else//create every odd layer
		{
			x_change = x_change - (gap * 1.4); //starts the first block on every layer to be at the edge, otherwise it would be created in the middle
			for(int k = 1; k< 4; k++)
			{	
				//this makes the even layer of boxes that stretch out across the z-axis and places 3 across
				CreateBox(otherBlocks,otherBlocks,blocklength, glm::vec3((x_change + (gap * (k * 0.7))), pos.y, z_change + (gap * 1.4)),glm::quat());
			}
		}
		pos.y += blocklength/4.75; // increase y for next layer to be on top
	}
}



void PhysicsFactory::CreateArm(float size, glm::vec3 position) //float size is used to set the base values and the sizes for every piece of the arm are based off this value
{
	float joint_change =0; //this is for creating appropriate spaces between joints

	for(int i = 0; i<size; i++) //joint change sets the hinge position of each block based on the user set size
	{
		joint_change += 0.5;
	}

	//These form the main part of the arm
	shared_ptr<PhysicsController> shoulder = CreateBox(size,size,size, glm::vec3(position.x, position.y, position.z), glm::quat());  //shoulder
	shared_ptr<PhysicsController> upperArm = CreateBox((size * 0.2),(size*0.8),(size * 0.2), glm::vec3(position.x, position.y - size, position.z), glm::quat());  //upper arm
	shared_ptr<PhysicsController> elbow = CreateBox((size*0.4),(size * 0.4),(size*0.4), glm::vec3(position.x, position.y - (size * 2), position.z), glm::quat());     //elbow
	shared_ptr<PhysicsController> lowerArm = CreateBox((size * 0.2),(size*0.8),(size * 0.2), glm::vec3(position.x,position.y - (size * 3), position.z), glm::quat());   //lower arm

	//hand and fingers
	//palm
	shared_ptr<PhysicsController> palm = CreateBox((size * 1.2),size,(size * 0.2), glm::vec3(position.x, position.y-(size*4), position.z), glm::quat());		//palm
	//thumb
	shared_ptr<PhysicsController> thumb = CreateBox((size * 0.4),(size*0.2),(size * 0.2), glm::vec3(position.x - size, position.y - (size * 4.2), position.z), glm::quat());		//thumb
	shared_ptr<PhysicsController> thumb_top = CreateBox((size * 0.4),(size*0.2),(size * 0.2), glm::vec3(position.x - (size * 1.6), position.y - (size * 4.28), position.z), glm::quat());//thumb top

	//1st finger
	shared_ptr<PhysicsController> finger1Lower = CreateBox((size * 0.2),(size*0.6),(size * 0.2), glm::vec3(position.x - (size /2), position.y-(size * 5), position.z), glm::quat());	//lower part of finger 1
	shared_ptr<PhysicsController> finger1mid = CreateBox((size * 0.2),(size*0.6),(size * 0.2), glm::vec3(position.x - (size/2), position.y - (size * 6), position.z), glm::quat());	//top part of finger 1
	
	//2nd finger
	shared_ptr<PhysicsController> finger2Lower = CreateBox((size * 0.2),(size*0.6),(size * 0.2), glm::vec3(position.x, position.y - (size * 5), position.z), glm::quat());	//lower part of finger 2
	shared_ptr<PhysicsController> finger2mid = CreateBox((size * 0.2),(size*0.6),(size * 0.2), glm::vec3(position.x, position.y - (size * 6), position.z), glm::quat());		//top part of finger 2
	
	//3rd finger
	shared_ptr<PhysicsController> finger3Lower = CreateBox((size * 0.2),(size*0.6),(size * 0.2), glm::vec3(position.x + (size / 2), position.y - (size * 5), position.z), glm::quat());	//lower part of finger 3
	shared_ptr<PhysicsController> finger3mid = CreateBox((size * 0.2),(size*0.6),(size * 0.2), glm::vec3(position.x + (size / 2), position.y - (size * 6), position.z), glm::quat());	//top part of finger 3
	

	//shoulder to upper arm - ball and socket
	btPoint2PointConstraint * shoulder_upArm = new btPoint2PointConstraint(*shoulder->rigidBody, *upperArm->rigidBody, btVector3(0,-joint_change,0), btVector3(0,joint_change,0));
	dynamicsWorld->addConstraint(shoulder_upArm);

	//conect upper arm to elbow - hinge
	btHingeConstraint * upperArm_elbow_hinge = new btHingeConstraint(*upperArm->rigidBody, *elbow->rigidBody, btVector3(0,-joint_change,0),btVector3(0,joint_change,0), btVector3(1,0,0), btVector3(1,0,0), true);
	upperArm_elbow_hinge->setLimit(2, 10, 0.9f, 0.01f, 0.0f);   //set limits
	dynamicsWorld->addConstraint(upperArm_elbow_hinge);
	
	//conect elbow to lower arm - hinge
	btHingeConstraint * lowerArm_elbow_hinge = new btHingeConstraint(*elbow->rigidBody, *lowerArm->rigidBody, btVector3(0,-joint_change,0),btVector3(0,joint_change,0), btVector3(1,0,0), btVector3(1,0,0), true);
	lowerArm_elbow_hinge->setLimit(2, 10, 0.9f, 0.01f, 0.0f);   //set limits
	dynamicsWorld->addConstraint(lowerArm_elbow_hinge);


	//thumb
	//conect thumb to palm - hinge
	btHingeConstraint * thumb_palm = new btHingeConstraint(*thumb->rigidBody, *palm->rigidBody, btVector3(joint_change,1.5f,0),btVector3(-joint_change,0,0), btVector3(1,0,0), btVector3(1,0,0), true);
	thumb_palm->setLimit(2, 10, 0.9f, 0.01f, 0.0f);   //set limits
	dynamicsWorld->addConstraint(thumb_palm);
	//conect thum to thumb_top - hinge
	btHingeConstraint * thumbFull = new btHingeConstraint(*thumb->rigidBody, *thumb_top->rigidBody, btVector3(-joint_change,0,0),btVector3(joint_change, 0,0), btVector3(1,0,0), btVector3(1,0,0), true);
	thumbFull->setLimit(2, 10, 0.9f, 0.01f, 0.0f);   //set limits
	dynamicsWorld->addConstraint(thumbFull);


	//conect finger1low to palm - hinge
	btHingeConstraint * finger1_palm = new btHingeConstraint(*palm->rigidBody, *finger1Lower->rigidBody, btVector3(-joint_change,-joint_change,0),btVector3(0, joint_change,0), btVector3(1,0,0), btVector3(1,0,0), true);
	finger1_palm->setLimit(2, 10, 0.9f, 0.01f, 0.0f);   //set limits
	dynamicsWorld->addConstraint(finger1_palm);
	//connect finger1mid to finger1low - hinge
	btHingeConstraint * finger1_Low_mid = new btHingeConstraint(*finger1Lower->rigidBody, *finger1mid->rigidBody, btVector3(0,-joint_change,0),btVector3(0, joint_change,0), btVector3(1,0,0), btVector3(1,0,0), true);
	finger1_Low_mid->setLimit(2, 10, 0.9f, 0.01f, 0.0f);   //set limits
	dynamicsWorld->addConstraint(finger1_Low_mid);


	//conect finger2low to palm - hinge
	btHingeConstraint * finger2_palm = new btHingeConstraint(*palm->rigidBody, *finger2Lower->rigidBody, btVector3(0,-joint_change,0),btVector3(0, joint_change,0), btVector3(1,0,0), btVector3(1,0,0), true);
	finger2_palm->setLimit(2, 10, 0.9f, 0.01f, 0.0f);   //set limits
	dynamicsWorld->addConstraint(finger2_palm);
	//connect finger2mid to finger2low - hinge
	btHingeConstraint * finger2_Low_mid = new btHingeConstraint(*finger2Lower->rigidBody, *finger2mid->rigidBody, btVector3(0,-joint_change,0),btVector3(0, joint_change,0), btVector3(1,0,0), btVector3(1,0,0), true);
	finger2_Low_mid->setLimit(2, 10, 0.9f, 0.01f, 0.0f);   //set limits
	dynamicsWorld->addConstraint(finger2_Low_mid);
	

	//conect finger3low to palm - hinge
	btHingeConstraint * finger3_palm = new btHingeConstraint(*palm->rigidBody, *finger3Lower->rigidBody, btVector3(joint_change,-joint_change,0),btVector3(0, joint_change,0), btVector3(1,0,0), btVector3(1,0,0), true);
	finger3_palm->setLimit(2, 10, 0.9f, 0.01f, 0.0f);   //set limits
	dynamicsWorld->addConstraint(finger3_palm);
	//connect finger3mid to finger3low - hinge
	btHingeConstraint * finger3_Low_mid = new btHingeConstraint(*finger3Lower->rigidBody, *finger3mid->rigidBody, btVector3(0,-joint_change,0),btVector3(0, joint_change,0), btVector3(1,0,0), btVector3(1,0,0), true);
	finger3_Low_mid->setLimit(2, 10, 0.9f, 0.01f, 0.0f);   //set limits
	dynamicsWorld->addConstraint(finger3_Low_mid);

	//lower arm to palm e.g wrist - ball and socket
	btPoint2PointConstraint * wrist = new btPoint2PointConstraint(*lowerArm->rigidBody, *palm->rigidBody, btVector3(0,-joint_change,0), btVector3(0,joint_change,0));
	dynamicsWorld->addConstraint(wrist);

}


shared_ptr<PhysicsController> PhysicsFactory::CreateFromModel(string name, glm::vec3 pos, glm::quat quat, glm::vec3 scale)
{
	shared_ptr<GameComponent> component = make_shared<GameComponent>();
	component->tag = name;
	component->scale = scale;
	Game::Instance()->Attach(component);
	shared_ptr<Model> model = Content::LoadModel(name);
	component->specular = glm::vec3(1.2f, 1.2f, 1.2f);
	model->Initialise();
	component->Attach(model);

	std::vector<glm::vec3>::iterator it = model->vertices.begin(); 	
	btConvexHullShape * tetraShape = new btConvexHullShape();

	while (it != model->vertices.end())
	{
		glm::vec4 point = glm::vec4(* it, 0) * glm::scale(glm::mat4(1), scale);
		tetraShape->addPoint(GLToBtVector(glm::vec3(point)));
		it ++;
	}
	
	btScalar mass = 1;
	btVector3 inertia(0,0,0);
	
	tetraShape->calculateLocalInertia(mass,inertia);
	btDefaultMotionState * motionState = new btDefaultMotionState(btTransform(GLToBtQuat(quat)
		,GLToBtVector(pos)));	
	
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass,motionState, tetraShape, inertia);
	btRigidBody * body = new btRigidBody(rigidBodyCI);
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	dynamicsWorld->addRigidBody(body);

	shared_ptr<PhysicsController> controller =make_shared<PhysicsController>(tetraShape, body, motionState);	
	body->setUserPointer(controller.get());
	component->Attach(controller);
	
	controller->tag = "Model";	
	return controller;
}

shared_ptr<PhysicsController> PhysicsFactory::CreateSphere(float radius, glm::vec3 pos, glm::quat quat)
{
	shared_ptr<GameComponent> sphere (new Sphere(radius));
	Game::Instance()->Attach(sphere);

	btDefaultMotionState * sphereMotionState = new btDefaultMotionState(btTransform(GLToBtQuat(quat)
		,GLToBtVector(pos)));	

	btScalar mass = 1;
	btVector3 sphereInertia(0,0,0);
	btCollisionShape * sphereShape = new btSphereShape(radius);

	sphereShape->calculateLocalInertia(mass,sphereInertia);
	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,sphereMotionState, sphereShape, sphereInertia);
	btRigidBody * body = new btRigidBody(fallRigidBodyCI);
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	dynamicsWorld->addRigidBody(body);

	shared_ptr<PhysicsController> sphereController (new PhysicsController(sphereShape, body, sphereMotionState));	
	body->setUserPointer(sphereController.get());
	sphere->Attach(sphereController);
	sphereController->tag = "Sphere";	
	return sphereController;
}


shared_ptr<PhysicsController> PhysicsFactory::CreateBox(float width, float height, float depth, glm::vec3 pos, glm::quat quat)
{
	// Create the shape
	btCollisionShape * boxShape = new btBoxShape(btVector3(width, height, depth) * 0.50);
	btScalar mass = 1;
	btVector3 boxInertia(0,0,0);
	boxShape->calculateLocalInertia(mass,boxInertia);

	// This is a container for the box model
	shared_ptr<Box> box = make_shared<Box>(width, height, depth);
	box->worldMode = GameComponent::from_child;
	box->position = pos;
	Game::Instance()->Attach(box);

	// Create the rigid body
	btDefaultMotionState * boxMotionState = new btDefaultMotionState(btTransform(GLToBtQuat(quat)
		,GLToBtVector(pos)));			
	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,  boxMotionState, boxShape, boxInertia);
	btRigidBody * body = new btRigidBody(fallRigidBodyCI);
	body->setFriction(567);
	dynamicsWorld->addRigidBody(body);

	// Create the physics component and add it to the box
	shared_ptr<PhysicsController> boxController = make_shared<PhysicsController>(PhysicsController(boxShape, body, boxMotionState));
	boxController->tag = "Box";
	body->setUserPointer(boxController.get());
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	box->Attach(boxController);

	return boxController;
}

shared_ptr<PhysicsController> PhysicsFactory::CreateCylinder(float radius, float height, glm::vec3 pos, glm::quat quat)
{
	// Create the shape
	btCollisionShape * shape = new btCylinderShape(btVector3(radius, height * 0.5f, radius));
	btScalar mass = 1;
	btVector3 inertia(0,0,0);
	shape->calculateLocalInertia(mass,inertia);

	// This is a container for the box model
	shared_ptr<GameComponent> cyl = make_shared<GameComponent>(Cylinder(radius, height));
	cyl->position = pos;
	Game::Instance()->Attach(cyl);

	// Create the rigid body
	btDefaultMotionState * motionState = new btDefaultMotionState(btTransform(GLToBtQuat(quat)
		,GLToBtVector(pos)));			
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass,  motionState, shape, inertia);
	btRigidBody * body = new btRigidBody(rigidBodyCI);
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	dynamicsWorld->addRigidBody(body);

	// Create the physics component and add it to the box
	shared_ptr<PhysicsController> component = make_shared<PhysicsController>(PhysicsController(shape, body, motionState));
	body->setUserPointer(component.get());
	component->tag = "Cylinder";
	cyl->Attach(component);

	return component;
}

shared_ptr<PhysicsController> PhysicsFactory::CreateCameraPhysics()
{
	btVector3 inertia;
	// Now add physics to the camera
	btCollisionShape * cameraCyl = new btCylinderShape(btVector3(0.5f, 5.0f, 2.5f));
	cameraCyl->calculateLocalInertia(1, inertia);
	shared_ptr<PhysicsCamera> physicsCamera = make_shared<PhysicsCamera>(this);

	shared_ptr<Camera> camera = Game::Instance()->camera;
	camera->Attach(physicsCamera);

	btRigidBody::btRigidBodyConstructionInfo cameraCI(10,physicsCamera.get(), cameraCyl, inertia);  
	btRigidBody * body = new btRigidBody(cameraCI);
	physicsCamera->SetPhysicsStuff(cameraCyl, body, physicsCamera.get());
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	body->setActivationState(DISABLE_DEACTIVATION);

	dynamicsWorld->addRigidBody(body);
	return physicsCamera;
}

shared_ptr<PhysicsController> PhysicsFactory::CreateVehicle(glm::vec3 position)
{
	float width = 15;
	float height = 2;
	float length = 5;
	float wheelWidth = 1;
	float wheelRadius = 2;
	float wheelOffset = 2.0f;

	shared_ptr<PhysicsController> chassis = CreateBox(width, height, length, position, glm::quat());

	shared_ptr<PhysicsController> wheel;
	glm::quat q =  glm::angleAxis(glm::half_pi<float>(), glm::vec3(1, 0, 0));

	glm::vec3 offset;
	btHingeConstraint * hinge;

	offset = glm::vec3(- (width / 2 - wheelRadius), 0, - (length / 2 + wheelOffset));
	wheel = CreateCylinder(wheelRadius, wheelWidth, position + offset, q);	 
	hinge = new btHingeConstraint(* chassis->rigidBody, * wheel->rigidBody, GLToBtVector(offset),btVector3(0,0, 0), btVector3(0,0,1), btVector3(0,1,0), true);
	dynamicsWorld->addConstraint(hinge);

	offset = glm::vec3(+ (width / 2 - wheelRadius), 0, - (length / 2 + wheelOffset));
	wheel = CreateCylinder(wheelRadius, wheelWidth, glm::vec3(position.x + (width / 2) - wheelRadius, position.y, position.z - (length / 2) - wheelWidth), q);
	hinge = new btHingeConstraint(* chassis->rigidBody, * wheel->rigidBody, GLToBtVector(offset),btVector3(0,0, 0), btVector3(0,0,1), btVector3(0,1,0), true);
	dynamicsWorld->addConstraint(hinge);

	offset = glm::vec3(- (width / 2 - wheelRadius), 0, + (length / 2 + wheelOffset));
	wheel = CreateCylinder(wheelRadius, wheelWidth, position + offset, q);	 
	hinge = new btHingeConstraint(* chassis->rigidBody, * wheel->rigidBody, GLToBtVector(offset),btVector3(0,0, 0), btVector3(0,0,1), btVector3(0,1,0), true);
	dynamicsWorld->addConstraint(hinge);

	offset = glm::vec3(+ (width / 2 - wheelRadius), 0, + (length / 2 + wheelOffset));
	wheel = CreateCylinder(wheelRadius, wheelWidth, position + offset, q);	 
	hinge = new btHingeConstraint(* chassis->rigidBody, * wheel->rigidBody, GLToBtVector(offset),btVector3(0,0, 0), btVector3(0,0,1), btVector3(0,1,0), true);
	dynamicsWorld->addConstraint(hinge);

	return chassis;
}

shared_ptr<PhysicsController> PhysicsFactory::CreateGroundPhysics()
{
	shared_ptr<Ground> ground = make_shared<Ground>();

	btCollisionShape * groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);
	btDefaultMotionState * groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,-1,0)));

	btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
	btRigidBody* body = new btRigidBody(groundRigidBodyCI);
	body->setFriction(100);
	dynamicsWorld->addRigidBody(body);
	body->setUserPointer(ground.get());
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	shared_ptr<PhysicsController> groundComponent (new PhysicsController(groundShape, body, groundMotionState));
	groundComponent->tag = "Ground";
	ground->Attach(groundComponent);	
	Game::Instance()->SetGround(ground);
	return groundComponent;
}

shared_ptr<PhysicsController> PhysicsFactory::CreateRandomObject(glm::vec3 point, glm::quat q)
{
	vector<string> names;
	DIR * dir;
	struct dirent * ent;
	dir = opendir (Content::prefix.c_str());
	if (dir != NULL) 
	{
		/* print all the files and directories within directory */
		while ((ent = readdir (dir)) != NULL) 
		{
			string fname = string(ent->d_name);
			int fpos = fname.find("objm");
			if (fpos != string::npos)
			{
				if ((fname.find("cube") == string::npos) && (fname.find("cyl") == string::npos) && (fname.find("sphere") == string::npos))
				{
					names.push_back(fname.substr(0, fpos - 1));
				}
			}
		}
		closedir (dir);
	} 
	else 
	{
		throw BGE::Exception("Could not list obj files in content folder");
	}

	int which = rand() % names.size();
	string name = names[which];
	return CreateFromModel(name, point, q, glm::vec3(3,3,3));
}
