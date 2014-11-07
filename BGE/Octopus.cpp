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


bool Octopus::Initialise()
{
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();

	shared_ptr<PhysicsController> box = physicsFactory->CreateBox(1, 1, 4, glm::vec3(5, 5, 0), glm::quat());

	setGravity();

	if (!Game::Initialise()) {
		return false;
	}

	return true;
}

void BGE::Octopus::Update(float timedelta)
{
	Game::Update(timedelta);
}

void BGE::Octopus::Cleanup()
{
	Game::Cleanup();
}