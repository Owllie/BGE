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
#include "Capsule.h"
using namespace BGE;

const float standardMass = 10.0f;

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

shared_ptr<PhysicsController> PhysicsFactory::CreateFromModel(string name, glm::vec3 pos, glm::quat quat, glm::vec3 scale)
{
	shared_ptr<GameComponent> component = make_shared<GameComponent>(true);
	component->tag = "Model";
	component->transform->scale = scale;
	Game::Instance()->Attach(component);
	shared_ptr<Model> model = Content::LoadModel(name);
	component->transform->specular = glm::vec3(1.2f, 1.2f, 1.2f);
	component->Attach(model);
	model->Initialise();

	std::vector<glm::vec3>::iterator it = model->vertices.begin(); 	
	btConvexHullShape * tetraShape = new btConvexHullShape();

	while (it != model->vertices.end())
	{
		glm::vec4 point = glm::vec4(* it, 0) * glm::scale(glm::mat4(1), scale);
		tetraShape->addPoint(GLToBtVector(glm::vec3(point)));
		it ++;
	}
	
	btScalar mass = standardMass;
	btVector3 inertia(0,0,0);
	
	tetraShape->calculateLocalInertia(mass,inertia);
	btDefaultMotionState * motionState = new btDefaultMotionState(btTransform(GLToBtQuat(quat)
		,GLToBtVector(pos)));	
	
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass,motionState, tetraShape, inertia);
	btRigidBody * body = new btRigidBody(rigidBodyCI);
	//body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	dynamicsWorld->addRigidBody(body);

	shared_ptr<PhysicsController> controller = make_shared<PhysicsController>(tetraShape, body, motionState);	
	body->setUserPointer(controller.get());
	component->Attach(controller);
	return controller;
}

shared_ptr<PhysicsController> PhysicsFactory::CreateSphere(float radius, glm::vec3 pos, glm::quat quat, bool kinematic, bool attachToGame)
{
	shared_ptr<GameComponent> sphere = make_shared<Sphere>(radius);
	sphere->Initialise();

	if (attachToGame)
	{
		Game::Instance()->Attach(sphere);
	}
	btDefaultMotionState * sphereMotionState = new btDefaultMotionState(btTransform(GLToBtQuat(quat)
		,GLToBtVector(pos)));	

	btScalar mass = (kinematic) ? 0 : standardMass;
	btVector3 sphereInertia(0,0,0);
	btCollisionShape * sphereShape = new btSphereShape(radius);

	sphereShape->calculateLocalInertia(mass,sphereInertia);
	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,sphereMotionState, sphereShape, sphereInertia);
	btRigidBody * body = new btRigidBody(fallRigidBodyCI);
	if (kinematic)
	{
		body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);		
	}
	body->setActivationState(DISABLE_DEACTIVATION);
	//body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	dynamicsWorld->addRigidBody(body);

	shared_ptr<PhysicsController> sphereController (new PhysicsController(sphereShape, body, sphereMotionState));	
	body->setUserPointer(sphereController.get());
	sphere->Attach(sphereController);
	sphereController->tag = "Sphere";	
	return sphereController;
}


shared_ptr<PhysicsController> PhysicsFactory::CreateBox(float width, float height, float depth, glm::vec3 pos, glm::quat quat, bool kinematic, bool attachToGame)
{
	// Create the shape
	btCollisionShape * boxShape = new btBoxShape(btVector3(width, height, depth) * 0.50);
	btScalar mass = (kinematic) ? 0 : standardMass;
	btVector3 boxInertia(0,0,0);
	boxShape->calculateLocalInertia(mass,boxInertia);

	// This is a container for the box model
	shared_ptr<Box> box = make_shared<Box>(width, height, depth);
	box->Initialise();
	box->transform->position = pos;
	cout << "CreateBox(): " << box -> transform -> position.x << ", " << box -> transform -> position.y << ", " << box -> transform -> position.z << endl; 
	box -> transform -> Calculate(); 

	if (attachToGame)
	{
		Game::Instance()->Attach(box);
	}
	// Create the rigid body
	btDefaultMotionState * boxMotionState = new btDefaultMotionState(btTransform(GLToBtQuat(quat)
		,GLToBtVector(pos)));			
	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,  boxMotionState, boxShape, boxInertia);
	btRigidBody * body = new btRigidBody(fallRigidBodyCI);
	body->setFriction(567);
	if (kinematic)
	{
		body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);		
	}
	body->setActivationState(DISABLE_DEACTIVATION);
	dynamicsWorld->addRigidBody(body);

	// Create the physics component and add it to the box
	shared_ptr<PhysicsController> boxController = make_shared<PhysicsController>(boxShape, body, boxMotionState);
	boxController->tag = "Box";
	body->setUserPointer(boxController.get());
	//body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	box->Attach(boxController);

	return boxController;
}

shared_ptr<PhysicsController> PhysicsFactory::CreateCylinder(float radius, float height, glm::vec3 pos, glm::quat quat, bool kinematic, bool attachToGame)
{
	// Create the shape
	btCollisionShape * shape = new btCylinderShape(btVector3(radius, height * 0.5f, radius));

	btScalar mass = (kinematic) ? 0 : standardMass;
	
	btVector3 inertia(0,0,0);
	shape->calculateLocalInertia(mass,inertia);

	// This is a container for the box model
	shared_ptr<GameComponent> cyl = make_shared<Cylinder>(radius, height);
	//cyl->Initialise();
	if (attachToGame)
	{
		Game::Instance()->Attach(cyl);
	}
	// Create the rigid body

	btTransform transform = btTransform(GLToBtQuat(quat), GLToBtVector(pos));
	glm::quat q = BtToGLQuat(transform.getRotation());
	btDefaultMotionState * motionState = new btDefaultMotionState(transform);			
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass,  motionState, shape, inertia);
	btRigidBody * body = new btRigidBody(rigidBodyCI);
	if (kinematic)
	{
		body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);		
	}
	body->setActivationState(DISABLE_DEACTIVATION);
	dynamicsWorld->addRigidBody(body);

	// Create the physics component and add it to the box
	shared_ptr<PhysicsController> component = make_shared<PhysicsController>(shape, body, motionState);
	body->setUserPointer(component.get());
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
	glm::quat q =  glm::angleAxis(90.0f, glm::vec3(1, 0, 0));

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
	hinge->enableAngularMotor(true, -10, 10);	
	dynamicsWorld->addConstraint(hinge);

	return chassis;
}

shared_ptr<PhysicsController> PhysicsFactory::CreateGroundPhysics()
{
	shared_ptr<Ground> ground = make_shared<Ground>();

	btCollisionShape * groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);
	btDefaultMotionState * groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));

	btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
	btRigidBody* body = new btRigidBody(groundRigidBodyCI);
	body->setFriction(100);
	dynamicsWorld->addRigidBody(body);
	body->setUserPointer(ground.get());
	//body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	shared_ptr<PhysicsController> groundComponent (new PhysicsController(groundShape, body, groundMotionState));
	groundComponent->tag = "Ground";
	Game::Instance()->SetGround(ground);
	ground->Attach(groundComponent);	
	return groundComponent;
}

shared_ptr<PhysicsController> PhysicsFactory::CreateRandomObject(glm::vec3 point, glm::quat q, glm::vec3 scale)
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
	return CreateFromModel(name, point, q, scale);
}

shared_ptr<PhysicsController> PhysicsFactory::CreateCapsule(float radius, float height, glm::vec3 pos, glm::quat quat)
{
	btCollisionShape* capsuleShape = new btCapsuleShape(btScalar(radius), btScalar(height));
	btScalar mass = 1;
	btVector3 inertia(0, 0, 0);
	capsuleShape->calculateLocalInertia(mass, inertia);

	// This is a container for the box model
	shared_ptr<GameComponent> cap = make_shared<Capsule>(radius, height);
	cap->Initialise();
	cap->transform->position = pos;
	Game::Instance()->Attach(cap);

	// Create the rigid body
	btDefaultMotionState * motionState = new btDefaultMotionState(btTransform(GLToBtQuat(quat)
		, GLToBtVector(pos)));
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass, motionState, capsuleShape, inertia);
	btRigidBody * body = new btRigidBody(rigidBodyCI);
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	dynamicsWorld->addRigidBody(body);

	// Create the physics component and add it to the box
	shared_ptr<PhysicsController> component = make_shared<PhysicsController>(capsuleShape, body, motionState);
	body->setUserPointer(component.get());
	component->tag = "capsule";
	cap->Attach(component);

	return component;
}

shared_ptr<PhysicsController> PhysicsFactory::CreateCapsuleRagdoll(glm::vec3 position)
{


	btHingeConstraint* spine_pelvis;
	btHingeConstraint* left_upper_leg_left_lower_leg;
	btHingeConstraint* right_upper_leg_right_lower_leg;
	btHingeConstraint* left_upper_arm_left_lower_arm;
	btHingeConstraint* right_upper_arm_right_lower_arm;

	btConeTwistConstraint* head_spine;
	btConeTwistConstraint* pelvis_left_upper_leg;
	btConeTwistConstraint* pelvis_right_upper_leg;
	btConeTwistConstraint* spine_left_upper_arm;
	btConeTwistConstraint* spine_right_upper_arm;

	shared_ptr<PhysicsController> bodypart_head = CreateCapsule(1.0, 0.8, glm::vec3(position.x, position.y + 20, position.z), glm::quat());
	shared_ptr<PhysicsController> bodypart_spine = CreateCapsule(1.8, 1.4, glm::vec3(position.x, position.y + 15, position.z), glm::quat());
	shared_ptr<PhysicsController> bodypart_pelvis = CreateCapsule(1.8, 0.8, glm::vec3(position.x, position.y + 10, position.z), glm::quat());

	shared_ptr<PhysicsController> bodypart_left_upper_leg = CreateCapsule(0.8, 1.4, glm::vec3(position.x - 1.5, position.y + 6, position.z), glm::quat());
	shared_ptr<PhysicsController> bodypart_left_lower_leg = CreateCapsule(0.6, 1.4, glm::vec3(position.x - 1.5, position.y, position.z), glm::quat());
	shared_ptr<PhysicsController> bodypart_right_upper_leg = CreateCapsule(0.8, 1.4, glm::vec3(position.x + 1.5, position.y + 6, position.z), glm::quat());
	shared_ptr<PhysicsController> bodypart_right_lower_leg = CreateCapsule(0.6, 1.4, glm::vec3(position.x + 1.5, position.y, position.z), glm::quat());

	shared_ptr<PhysicsController> bodypart_left_upper_arm = CreateCapsule(0.6, 1.2, glm::vec3(position.x - 3, position.y + 14, position.z), glm::quat());
	shared_ptr<PhysicsController> bodypart_left_lower_arm = CreateCapsule(0.5, 1.2, glm::vec3(position.x - 3, position.y + 9, position.z), glm::quat());
	shared_ptr<PhysicsController> bodypart_right_upper_arm = CreateCapsule(0.6, 1.2, glm::vec3(position.x + 3, position.y + 14, position.z), glm::quat());
	shared_ptr<PhysicsController> bodypart_right_lower_arm = CreateCapsule(0.5, 1.2, glm::vec3(position.x + 3, position.y + 9, position.z), glm::quat());

	btTransform localA, localB;

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0, glm::half_pi<float>(), 0); localA.setOrigin(btVector3(btScalar(0.), btScalar(2.7), btScalar(0.)));
	localB.getBasis().setEulerZYX(0, glm::half_pi<float>(), 0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-2.7), btScalar(0.)));
	spine_pelvis = new btHingeConstraint(*bodypart_pelvis->rigidBody, *bodypart_spine->rigidBody, localA, localB);
	spine_pelvis->setLimit(btScalar(-glm::half_pi<float>()), btScalar(glm::half_pi<float>()));
	dynamicsWorld->addConstraint(spine_pelvis);

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0, 0, glm::half_pi<float>()); localA.setOrigin(btVector3(btScalar(0.), btScalar(2.7), btScalar(0.)));
	localB.getBasis().setEulerZYX(0, 0, glm::half_pi<float>()); localB.setOrigin(btVector3(btScalar(0.), btScalar(-2.7), btScalar(0.)));
	head_spine = new btConeTwistConstraint(*bodypart_spine->rigidBody, *bodypart_head->rigidBody, localA, localB);
	head_spine->setLimit(glm::quarter_pi<float>(), glm::quarter_pi<float>(), glm::half_pi<float>());
	dynamicsWorld->addConstraint(head_spine);

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0, 0, -glm::quarter_pi<float>() * 5); 
	localA.setOrigin(btVector3(btScalar(-1.5), btScalar(-2.5), btScalar(0.)));
	localB.getBasis().setEulerZYX(0, 0, -glm::quarter_pi<float>() * 5); 
	localB.setOrigin(btVector3(btScalar(0.), btScalar(2.5), btScalar(0.)));
	pelvis_left_upper_leg = new btConeTwistConstraint(*bodypart_pelvis->rigidBody, *bodypart_left_upper_leg->rigidBody, localA, localB);
	pelvis_left_upper_leg->setLimit(glm::quarter_pi<float>(), glm::half_pi<float>(), 0);
	dynamicsWorld->addConstraint(pelvis_left_upper_leg);

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0, glm::half_pi<float>(), 0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-2.7), btScalar(0.)));
	localB.getBasis().setEulerZYX(0, glm::half_pi<float>(), 0); localB.setOrigin(btVector3(btScalar(0.), btScalar(2.7), btScalar(0.)));
	left_upper_leg_left_lower_leg = new btHingeConstraint(*bodypart_left_upper_leg->rigidBody, *bodypart_left_lower_leg->rigidBody, localA, localB);
	left_upper_leg_left_lower_leg->setLimit(btScalar(-glm::half_pi<float>()), btScalar(0));
	dynamicsWorld->addConstraint(left_upper_leg_left_lower_leg);

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0, 0, glm::quarter_pi<float>()); localA.setOrigin(btVector3(btScalar(1.5), btScalar(-2.5), btScalar(0.)));
	localB.getBasis().setEulerZYX(0, 0, glm::quarter_pi<float>()); localB.setOrigin(btVector3(btScalar(0.), btScalar(2.5), btScalar(0.)));
	pelvis_right_upper_leg = new btConeTwistConstraint(*bodypart_pelvis->rigidBody, *bodypart_right_upper_leg->rigidBody, localA, localB);
	pelvis_right_upper_leg->setLimit(glm::quarter_pi<float>(), glm::half_pi<float>(), 0);
	dynamicsWorld->addConstraint(pelvis_right_upper_leg);

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0, glm::half_pi<float>(), 0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-2.7), btScalar(0.)));
	localB.getBasis().setEulerZYX(0, glm::half_pi<float>(), 0); localB.setOrigin(btVector3(btScalar(0.), btScalar(2.7), btScalar(0.)));
	right_upper_leg_right_lower_leg = new btHingeConstraint(*bodypart_right_upper_leg->rigidBody, *bodypart_right_lower_leg->rigidBody, localA, localB);
	right_upper_leg_right_lower_leg->setLimit(btScalar(-glm::half_pi<float>()), btScalar(0));
	dynamicsWorld->addConstraint(right_upper_leg_right_lower_leg);

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0, 0, M_PI); localA.setOrigin(btVector3(btScalar(-2.5), btScalar(2.5), btScalar(0.)));
	localB.getBasis().setEulerZYX(0, 0, glm::half_pi<float>()); localB.setOrigin(btVector3(btScalar(0.), btScalar(-2.5), btScalar(0.)));
	spine_left_upper_arm = new btConeTwistConstraint(*bodypart_spine->rigidBody, *bodypart_left_upper_arm->rigidBody, localA, localB);
	spine_left_upper_arm->setLimit(M_PI, M_PI, 0);
	dynamicsWorld->addConstraint(spine_left_upper_arm);

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0, glm::half_pi<float>(), 0); localA.setOrigin(btVector3(btScalar(0.), btScalar(2.5), btScalar(0.)));
	localB.getBasis().setEulerZYX(0, glm::half_pi<float>(), 0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-2.5), btScalar(0.)));
	left_upper_arm_left_lower_arm = new btHingeConstraint(*bodypart_left_upper_arm->rigidBody, *bodypart_left_lower_arm->rigidBody, localA, localB);
	left_upper_arm_left_lower_arm->setLimit(btScalar(-glm::half_pi<float>()), btScalar(0));
	dynamicsWorld->addConstraint(left_upper_arm_left_lower_arm);

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0, 0, 0); localA.setOrigin(btVector3(btScalar(2.5), btScalar(2.5), btScalar(0.)));
	localB.getBasis().setEulerZYX(0, 0, glm::half_pi<float>()); localB.setOrigin(btVector3(btScalar(0.), btScalar(-2.5), btScalar(0.)));
	spine_right_upper_arm = new btConeTwistConstraint(*bodypart_spine->rigidBody, *bodypart_right_upper_arm->rigidBody, localA, localB);
	spine_right_upper_arm->setLimit(M_PI, M_PI, 0);
	dynamicsWorld->addConstraint(spine_right_upper_arm);

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0, glm::half_pi<float>(), 0); localA.setOrigin(btVector3(btScalar(0.), btScalar(2.5), btScalar(0.)));
	localB.getBasis().setEulerZYX(0, glm::half_pi<float>(), 0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-2.5), btScalar(0.)));
	right_upper_arm_right_lower_arm = new btHingeConstraint(*bodypart_right_upper_arm->rigidBody, *bodypart_right_lower_arm->rigidBody, localA, localB);
	right_upper_arm_right_lower_arm->setLimit(btScalar(-glm::half_pi<float>()), btScalar(0));
	dynamicsWorld->addConstraint(right_upper_arm_right_lower_arm);

	return bodypart_spine;
}

//Assignment attempt, The function only takes in the position of where you would like top spwan the octopus
shared_ptr<PhysicsController> PhysicsFactory::CreateOctopus(glm::vec3 position)
{

	//Create body and mandibles, made of spheres and capsules
	shared_ptr<PhysicsController> body = CreateSphere(10.0f,
		glm::vec3(position.x, position.y + 5, position.z),
		glm::quat());

	shared_ptr<PhysicsController> mandible_1_BJ = CreateSphere(1.5f,
		glm::vec3(position.x + 2, position.y + 0, position.z + 2),
		glm::quat());

	shared_ptr<PhysicsController> manidible_1_sec_1 = CreateCapsule(1.5, 1.75,
		glm::vec3(10, 0, 10),
		glm::angleAxis(90.0f, glm::vec3(position.x + - 1, position.y, position.z + 1)));

	shared_ptr<PhysicsController> manidible_1_sec_2 = CreateSphere(1.5f,
		glm::vec3(position.x + 14, position.y, position.z + 14),
		glm::quat());

	shared_ptr<PhysicsController> manidible_1_sec_3 = CreateCapsule(1.5, 1.75,
		glm::vec3(position.x + 18, position.y, position.z + 18),
		glm::angleAxis(90.0f, glm::vec3(-1, 0, 1)));

	shared_ptr<PhysicsController> manidible_1_sec_4 = CreateSphere(1.5f,
		glm::vec3(position.x + 22, position.y, position.z + 22),
		glm::quat());



	shared_ptr<PhysicsController> mandible_2_BJ = CreateSphere(1.5f,
		glm::vec3(position.x - 2, position.y, position.z - 2),
		glm::quat());

	shared_ptr<PhysicsController> manidible_2_sec_1 = CreateCapsule(1.5, 1.75,
		glm::vec3(position.x - 10, position.y, position.z - 10),
		glm::angleAxis(90.0f, glm::vec3(-1, 0, 1)));

	shared_ptr<PhysicsController> manidible_2_sec_2 = CreateSphere(1.5f,
		glm::vec3(position.x - 14, position.y, position.z- 14),
		glm::quat());

	shared_ptr<PhysicsController> manidible_2_sec_3 = CreateCapsule(1.5, 1.75,
		glm::vec3(position.x - 18, position.y, position.z - 18),
		glm::angleAxis(90.0f, glm::vec3(-1, 0, 1)));

	shared_ptr<PhysicsController> manidible_2_sec_4 = CreateSphere(1.5f,
		glm::vec3(position.x - 22, position.y, position.z - 22),
		glm::quat());



	shared_ptr<PhysicsController> mandible_3_BJ = CreateSphere(1.5f,
		glm::vec3(position.x + 2, position.y, position.z - 2),
		glm::quat());

	shared_ptr<PhysicsController> manidible_3_sec_1 = CreateCapsule(1.5, 1.75,
		glm::vec3(position.x + 10, position.y, position.z - 10),
		glm::angleAxis(90.0f, glm::vec3(-1, 0, -1)));

	shared_ptr<PhysicsController> manidible_3_sec_2 = CreateSphere(1.5f,
		glm::vec3(position.x + 14, position.y, position.z - 14),
		glm::quat());

	shared_ptr<PhysicsController> manidible_3_sec_3 = CreateCapsule(1.5, 1.75,
		glm::vec3(position.x + 18, position.y, position.z - 18),
		glm::angleAxis(90.0f, glm::vec3(-1, 0, -1)));

	shared_ptr<PhysicsController> manidible_3_sec_4 = CreateSphere(1.5f,
		glm::vec3(position.x + 22, position.y, position.z - 22),
		glm::quat());



	shared_ptr<PhysicsController> mandible_4_BJ = CreateSphere(1.5f,
		glm::vec3(position.x - 2, position.y, position.z + 2),
		glm::quat());

	shared_ptr<PhysicsController> manidible_4_sec_1 = CreateCapsule(1.5, 1.75,
		glm::vec3(position.x - 10, position.y, position.z + 10),
		glm::angleAxis(90.0f, glm::vec3(-1, 0, -1)));

	shared_ptr<PhysicsController> manidible_4_sec_2 = CreateSphere(1.5f,
		glm::vec3(position.x - 14, position.y, position.z + 14),
		glm::quat());

	shared_ptr<PhysicsController> manidible_4_sec_3 = CreateCapsule(1.5, 1.75,
		glm::vec3(position.x - 18, position.y, position.z + 18),
		glm::angleAxis(90.0f, glm::vec3(-1, 0, -1)));

	shared_ptr<PhysicsController> manidible_4_sec_4 = CreateSphere(1.5f,
		glm::vec3(position.x - 22, position.y, position.z + 22),
		glm::quat());



	shared_ptr<PhysicsController> mandible_5_BJ = CreateSphere(1.5f,
		glm::vec3(position.x, position.y, position.z - 2),
		glm::quat());

	shared_ptr<PhysicsController> manidible_5_sec_1 = CreateCapsule(1.5, 1.75,
		glm::vec3(position.x, position.y, position.z - 15),
		glm::angleAxis(90.0f, glm::vec3(1, 0, 0)));

	shared_ptr<PhysicsController> manidible_5_sec_2 = CreateSphere(1.5f,
		glm::vec3(position.x, position.y, position.z - 21),
		glm::quat());

	shared_ptr<PhysicsController> manidible_5_sec_3 = CreateCapsule(1.5, 1.75,
		glm::vec3(position.x, position.y, position.z - 27),
		glm::angleAxis(90.0f, glm::vec3(1, 0, 0)));

	shared_ptr<PhysicsController> manidible_5_sec_4 = CreateSphere(1.5f,
		glm::vec3(position.x, position.y, position.z - 33),
		glm::quat());



	shared_ptr<PhysicsController> mandible_6_BJ = CreateSphere(1.5f,
		glm::vec3(position.x, position.y, position.z + 2),
		glm::quat());

	shared_ptr<PhysicsController> manidible_6_sec_1 = CreateCapsule(1.5, 1.75,
		glm::vec3(position.x, position.y, position.z + 15),
		glm::angleAxis(90.0f, glm::vec3(1, 0, 0)));

	shared_ptr<PhysicsController> manidible_6_sec_2 = CreateSphere(1.5f,
		glm::vec3(position.x, position.y, position.z + 21),
		glm::quat());

	shared_ptr<PhysicsController> manidible_6_sec_3 = CreateCapsule(1.5, 1.75,
		glm::vec3(position.x, position.y, position.z + 27),
		glm::angleAxis(90.0f, glm::vec3(1, 0, 0)));

	shared_ptr<PhysicsController> manidible_6_sec_4 = CreateSphere(1.5f,
		glm::vec3(position.x, position.y, position.z + 33),
		glm::quat());



	shared_ptr<PhysicsController> mandible_7_BJ = CreateSphere(1.5f,
		glm::vec3(position.x + 2, position.y, position.z),
		glm::quat());

	shared_ptr<PhysicsController> manidible_7_sec_1 = CreateCapsule(1.5, 1.75,
		glm::vec3(position.x + 15, position.y, position.z),
		glm::angleAxis(90.0f, glm::vec3(0, 0, 1)));

	shared_ptr<PhysicsController> manidible_7_sec_2 = CreateSphere(1.5f,
		glm::vec3(position.x + 21, position.y, position.z),
		glm::quat());

	shared_ptr<PhysicsController> manidible_7_sec_3 = CreateCapsule(1.5, 1.75,
		glm::vec3(position.x + 27, position.y, position.z),
		glm::angleAxis(90.0f, glm::vec3(0, 0, 1)));

	shared_ptr<PhysicsController> manidible_7_sec_4 = CreateSphere(1.5f,
		glm::vec3(position.x + 33, position.y, position.z),
		glm::quat());



	shared_ptr<PhysicsController> mandible_8_BJ = CreateSphere(1.5f,
		glm::vec3(position.x - 2, position.y, position.z),
		glm::quat());

	shared_ptr<PhysicsController> manidible_8_sec_1 = CreateCapsule(1.5, 1.75,
		glm::vec3(position.x - 15, position.y, position.z),
		glm::angleAxis(90.0f, glm::vec3(0, 0, 1)));

	shared_ptr<PhysicsController> manidible_8_sec_2 = CreateSphere(1.5f,
		glm::vec3(position.x - 21, position.y, position.z),
		glm::quat());

	shared_ptr<PhysicsController> manidible_8_sec_3 = CreateCapsule(1.5, 1.75,
		glm::vec3(position.x - 27, position.y, position.z),
		glm::angleAxis(90.0f, glm::vec3(0, 0, 1)));

	shared_ptr<PhysicsController> manidible_8_sec_4 = CreateSphere(1.5f,
		glm::vec3(position.x - 33, position.y, position.z),
		glm::quat());


	//Hinges joints being created
	btHingeConstraint * hinge33 = new btHingeConstraint(*mandible_1_BJ->rigidBody, *body->rigidBody, btVector3(btScalar(1), btScalar(1), btScalar(1)), btVector3(btScalar(4), btScalar(-8.5), btScalar(4)), btVector3(1, 1, 1), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge33);
	btHingeConstraint * hinge34 = new btHingeConstraint(*mandible_2_BJ->rigidBody, *body->rigidBody, btVector3(btScalar(-1), btScalar(1), btScalar(-1)), btVector3(btScalar(-4), btScalar(-8.5), btScalar(-4)), btVector3(-1, 1, -1), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge34);
	btHingeConstraint * hinge35 = new btHingeConstraint(*mandible_3_BJ->rigidBody, *body->rigidBody, btVector3(btScalar(1), btScalar(1), btScalar(-1)), btVector3(btScalar(4), btScalar(-8.5), btScalar(-4)), btVector3(1, 1, -1), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge35);
	btHingeConstraint * hinge36 = new btHingeConstraint(*mandible_4_BJ->rigidBody, *body->rigidBody, btVector3(btScalar(-1), btScalar(1), btScalar(1)), btVector3(btScalar(-4), btScalar(-8.5), btScalar(4)), btVector3(-1, 1, 1), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge36);
	btHingeConstraint * hinge37 = new btHingeConstraint(*mandible_5_BJ->rigidBody, *body->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(-1)), btVector3(btScalar(0), btScalar(-8.5), btScalar(-4)), btVector3(0, 1, -1), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge37);
	btHingeConstraint * hinge38 = new btHingeConstraint(*mandible_6_BJ->rigidBody, *body->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(1)), btVector3(btScalar(0), btScalar(-8.5), btScalar(4)), btVector3(0, 1, 1), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge38);
	btHingeConstraint * hinge39 = new btHingeConstraint(*mandible_7_BJ->rigidBody, *body->rigidBody, btVector3(btScalar(1), btScalar(1), btScalar(0)), btVector3(btScalar(4), btScalar(-8.5), btScalar(0)), btVector3(1, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge39);
	btHingeConstraint * hinge40 = new btHingeConstraint(*mandible_8_BJ->rigidBody, *body->rigidBody, btVector3(btScalar(-1), btScalar(1), btScalar(0)), btVector3(btScalar(-4), btScalar(-8.5), btScalar(0)), btVector3(-1, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge40);

	btHingeConstraint * hinge = new btHingeConstraint(*mandible_1_BJ->rigidBody, *manidible_1_sec_1->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 0, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge);
	btHingeConstraint * hinge2 = new btHingeConstraint(*manidible_1_sec_1->rigidBody, *manidible_1_sec_2->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge2);
	btHingeConstraint * hinge3 = new btHingeConstraint(*manidible_1_sec_2->rigidBody, *manidible_1_sec_3->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge3);
	btHingeConstraint * hinge4 = new btHingeConstraint(*manidible_1_sec_3->rigidBody, *manidible_1_sec_4->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge4);

	btHingeConstraint * hinge5 = new btHingeConstraint(*mandible_2_BJ->rigidBody, *manidible_2_sec_1->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 0, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge5);
	btHingeConstraint * hinge6 = new btHingeConstraint(*manidible_2_sec_1->rigidBody, *manidible_2_sec_2->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge6);
	btHingeConstraint * hinge7 = new btHingeConstraint(*manidible_2_sec_2->rigidBody, *manidible_2_sec_3->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge7);
	btHingeConstraint * hinge8 = new btHingeConstraint(*manidible_2_sec_3->rigidBody, *manidible_2_sec_4->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge8);

	btHingeConstraint * hinge9 = new btHingeConstraint(*mandible_3_BJ->rigidBody, *manidible_3_sec_1->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 0, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge9);
	btHingeConstraint * hinge10 = new btHingeConstraint(*manidible_3_sec_1->rigidBody, *manidible_3_sec_2->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge10);
	btHingeConstraint * hinge11 = new btHingeConstraint(*manidible_3_sec_2->rigidBody, *manidible_3_sec_3->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge11);
	btHingeConstraint * hinge12 = new btHingeConstraint(*manidible_3_sec_3->rigidBody, *manidible_3_sec_4->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge12);

	btHingeConstraint * hinge13 = new btHingeConstraint(*mandible_4_BJ->rigidBody, *manidible_4_sec_1->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 0, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge13);
	btHingeConstraint * hinge14 = new btHingeConstraint(*manidible_4_sec_1->rigidBody, *manidible_4_sec_2->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge14);
	btHingeConstraint * hinge15 = new btHingeConstraint(*manidible_4_sec_2->rigidBody, *manidible_4_sec_3->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge15);
	btHingeConstraint * hinge16 = new btHingeConstraint(*manidible_4_sec_3->rigidBody, *manidible_4_sec_4->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge16);

	btHingeConstraint * hinge17 = new btHingeConstraint(*mandible_5_BJ->rigidBody, *manidible_5_sec_1->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 0, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge17);
	btHingeConstraint * hinge18 = new btHingeConstraint(*manidible_5_sec_1->rigidBody, *manidible_5_sec_2->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge18);
	btHingeConstraint * hinge19 = new btHingeConstraint(*manidible_5_sec_2->rigidBody, *manidible_5_sec_3->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge19);
	btHingeConstraint * hinge28 = new btHingeConstraint(*manidible_5_sec_3->rigidBody, *manidible_5_sec_4->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge28);

	btHingeConstraint * hinge20 = new btHingeConstraint(*mandible_6_BJ->rigidBody, *manidible_6_sec_1->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 0, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge20);
	btHingeConstraint * hinge21 = new btHingeConstraint(*manidible_6_sec_1->rigidBody, *manidible_6_sec_2->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge21);
	btHingeConstraint * hinge22 = new btHingeConstraint(*manidible_6_sec_2->rigidBody, *manidible_6_sec_3->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge22);
	btHingeConstraint * hinge23 = new btHingeConstraint(*manidible_6_sec_3->rigidBody, *manidible_6_sec_4->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge23);

	btHingeConstraint * hinge24 = new btHingeConstraint(*mandible_7_BJ->rigidBody, *manidible_7_sec_1->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 0, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge24);
	btHingeConstraint * hinge25 = new btHingeConstraint(*manidible_7_sec_1->rigidBody, *manidible_7_sec_2->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge25);
	btHingeConstraint * hinge26 = new btHingeConstraint(*manidible_7_sec_2->rigidBody, *manidible_7_sec_3->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge26);
	btHingeConstraint * hinge27 = new btHingeConstraint(*manidible_7_sec_3->rigidBody, *manidible_7_sec_4->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge27);

	btHingeConstraint * hinge29 = new btHingeConstraint(*mandible_8_BJ->rigidBody, *manidible_8_sec_1->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 0, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge29);
	btHingeConstraint * hinge30 = new btHingeConstraint(*manidible_8_sec_1->rigidBody, *manidible_8_sec_2->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge30);
	btHingeConstraint * hinge31 = new btHingeConstraint(*manidible_8_sec_2->rigidBody, *manidible_8_sec_3->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge31);
	btHingeConstraint * hinge32 = new btHingeConstraint(*manidible_8_sec_3->rigidBody, *manidible_8_sec_4->rigidBody, btVector3(btScalar(0), btScalar(1), btScalar(0)), btVector3(btScalar(0), btScalar(-4), btScalar(0)), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge32);

	body->rigidBody->setMassProps(btScalar(4), btVector3(0, 0, 0));
	//setting limits so they aren't comletely uncontrollable
	hinge33->setLimit(btScalar(0), btScalar(0));
	hinge34->setLimit(btScalar(0), btScalar(0));
	hinge35->setLimit(btScalar(0), btScalar(0));
	hinge36->setLimit(btScalar(0), btScalar(0));
	hinge37->setLimit(btScalar(0), btScalar(0));
	hinge38->setLimit(btScalar(0), btScalar(0));
	hinge39->setLimit(btScalar(0), btScalar(0));
	hinge40->setLimit(btScalar(0), btScalar(0));
	//attaempt at enabling motors
	hinge33->enableAngularMotor(true, 10, 10);
	hinge34->enableAngularMotor(true, 10, 10);
	hinge35->enableAngularMotor(true, 10, 10);
	hinge36->enableAngularMotor(true, 10, 10);
	hinge37->enableAngularMotor(true, 10, 10);
	hinge38->enableAngularMotor(true, 10, 10);
	hinge39->enableAngularMotor(true, 10, 10);
	hinge40->enableAngularMotor(true, 10, 10);
	return body;
}