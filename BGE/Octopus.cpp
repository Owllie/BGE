#include "Octopus.h"
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

Octopus::Octopus(void)
{
}


Octopus::~Octopus(void)
{
}

shared_ptr<PhysicsController> octopus;

bool Octopus::Initialise()
{

	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();

	setGravity(glm::vec3(0, -9, 0));

	if (!Game::Initialise()) {
		return false;
	}

	glm::vec3 position = glm::vec3(2,2,2);

	physicsFactory->CreateOctopus(position);
	
}

void BGE::Octopus::Update(float timedelta)
{
	Game::Update();
}

void BGE::Octopus::Cleanup()
{
	Game::Cleanup();
}
