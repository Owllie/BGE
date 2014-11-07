#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

namespace BGE
{
	class Octopus:
		public Game
	{
	private:

	public:
		Octopus(void);
		~Octopus(void);
		bool Initialise();
		void Update(float timeDelta);
		void Cleanup();
		void CreateWall();
	};
}

