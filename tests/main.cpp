#define CATCH_CONFIG_MAIN
#include <algorithm>

#include "catch.hpp"
#include <btBulletDynamicsCommon.h>
#include "btInfCollisionDispatcher.h"
#include "btInfRigidBody.h"
#include "btInfBroadphase.h"

SCENARIO("Sphere-sphere collision") 
{
	const btScalar tileSize = 10;
	auto broadphase = std::make_unique<btInfBroadphase>(tileSize);
	auto collision_config = std::make_unique<btDefaultCollisionConfiguration>();
	auto dispatcher = std::make_unique<btInfCollisionDispatcher>(collision_config.get(), tileSize);
	auto solver = std::make_unique<btSequentialImpulseConstraintSolver>();
	auto world = std::make_unique<btDiscreteDynamicsWorld>(
		dispatcher.get(), broadphase.get(), solver.get(), collision_config.get()
	);

	world->setGravity(btVector3(0,0,0));

	double sphere_mass = 1;
	double sphere_restitution = 0.98;
	btQuaternion sphere_orientation{0, 0, 0, 1};
	btVector3 sphere_inertia;
	auto sphere_shape = std::make_unique<btSphereShape>(1);
	sphere_shape->calculateLocalInertia(sphere_mass, sphere_inertia);

	std::unique_ptr<btInfRigidBody> sphere1_body;
	std::unique_ptr<btInfRigidBody> sphere2_body;

	auto sphere_info = [&]() {
		std::stringstream ss;
		ss << "sphere1 pos(" << sphere1_body->getCenterOfMassPosition().x() <<
			", " << sphere1_body->getCenterOfMassPosition().y() <<
			", " << sphere1_body->getCenterOfMassPosition().z() << ") " <<
			" vel(" << sphere1_body->getLinearVelocity().x() <<
			", " << sphere1_body->getLinearVelocity().y() <<
			", " << sphere1_body->getLinearVelocity().z() << ") " <<
			"; sphere2 pos(" << sphere2_body->getCenterOfMassPosition().x() <<
			", " << sphere2_body->getCenterOfMassPosition().y() <<
			", " << sphere2_body->getCenterOfMassPosition().z() << ") " <<
			" vel(" << sphere2_body->getLinearVelocity().x() <<
			", " << sphere2_body->getLinearVelocity().y() <<
			", " << sphere2_body->getLinearVelocity().z() << ") ";
		return ss.str();
	};

	auto create_sphere = [&](const btVector3& tile, const btVector3& offset) {
		btRigidBody::btRigidBodyConstructionInfo sphere_construct{
			sphere_mass, nullptr, sphere_shape.get(), sphere_inertia
		};
		sphere_construct.m_startWorldTransform = btTransform{
			sphere_orientation, offset
		};
		sphere_construct.m_restitution = sphere_restitution;
		auto sphere_body = std::make_unique<btInfRigidBody>(sphere_construct, tile);
		world->addRigidBody(sphere_body.get());	
		return sphere_body;
	};

	auto step_simulation = [&](const double dt) {
		world->stepSimulation(dt, std::max(1, int(60*dt + 0.5)), 1.0/60);
	};

	GIVEN("Two spheres in the same tile")
	{
		sphere1_body = create_sphere(btVector3{1,2,3}, btVector3{2, 2, 0});
		sphere2_body = create_sphere(btVector3{1,2,3}, btVector3{-2, -2, 0});

		WHEN("spheres are sent flying toward one-another")
		{
			sphere1_body->applyCentralImpulse(btVector3(-1, -1, 0));
			sphere2_body->applyCentralImpulse(btVector3(1, 1, 0));

			AND_WHEN("simulation is stepped until just before the collision")
			{
				INFO(sphere_info());
				step_simulation(1.0);
				INFO(sphere_info());
				THEN("direction of travel remains the same")
				{
					CHECK(sphere1_body->getLinearVelocity().x() == -1);
					CHECK(sphere1_body->getLinearVelocity().y() == -1);
					CHECK(sphere1_body->getLinearVelocity().z() == 0);
					CHECK(sphere2_body->getLinearVelocity().x() == 1);
					CHECK(sphere2_body->getLinearVelocity().y() == 1);
					CHECK(sphere2_body->getLinearVelocity().z() == 0);
				}

				AND_WHEN("simulation is stepped to just after the collision")
				{
					INFO(sphere_info());
					step_simulation(0.35);

					THEN("spheres have rebounded off one-another")
					{
						INFO(sphere_info());
						CHECK(sphere1_body->getLinearVelocity().x() == Approx(1).epsilon(0.05));
						CHECK(sphere1_body->getLinearVelocity().y() == Approx(1).epsilon(0.05));
						CHECK(sphere1_body->getLinearVelocity().z() == 0);
						CHECK(sphere2_body->getLinearVelocity().x() == Approx(-1).epsilon(0.05));
						CHECK(sphere2_body->getLinearVelocity().y() == Approx(-1).epsilon(0.05));
						CHECK(sphere2_body->getLinearVelocity().z() == 0);
					}
				}
			}
		}
	}
	GIVEN("Two spheres in neighbouring tiles")
	{
		sphere1_body = create_sphere(btVector3{1,2,3}, btVector3{2, 2, 0});
		sphere2_body = create_sphere(btVector3{0,1,3}, btVector3{-2, -2, 0});

		WHEN("spheres are sent flying toward one-another")
		{
			sphere1_body->applyCentralImpulse(btVector3(-1, -1, 0));
			sphere2_body->applyCentralImpulse(btVector3(1, 1, 0));

			AND_WHEN("simulation is stepped a little")
			{
				INFO(sphere_info());
				step_simulation(1.0/60);
				INFO(sphere_info());

				THEN("broadphase has not flagged potential overlap")
				{
					CHECK(
						broadphase->getOverlappingPairCache()->getNumOverlappingPairs()
						== 0
					);
				}
			}

			AND_WHEN("simulation is stepped until collision would occur if in same tile")
			{
				INFO(sphere_info());
				step_simulation(1.35);
				INFO(sphere_info());
				THEN("direction of travel remains the same")
				{
					CHECK(sphere1_body->getLinearVelocity().x() == -1);
					CHECK(sphere1_body->getLinearVelocity().y() == -1);
					CHECK(sphere1_body->getLinearVelocity().z() == 0);
					CHECK(sphere2_body->getLinearVelocity().x() == 1);
					CHECK(sphere2_body->getLinearVelocity().y() == 1);
					CHECK(sphere2_body->getLinearVelocity().z() == 0);
				}
				THEN("broadphase has not flagged potential overlap")
				{
					CHECK(
						broadphase->getOverlappingPairCache()->getNumOverlappingPairs()
						== 0
					);
				}				
			}

			AND_WHEN("simulation is stepped until just before collision")
			{
				INFO(sphere_info());
				step_simulation(5.9);
				step_simulation(0.2);
				INFO(sphere_info());
				THEN("broadphase has flagged a potential overlap")
				{
					CHECK(
						broadphase->getOverlappingPairCache()->getNumOverlappingPairs()
						== 1
					);
				}	
			}

			AND_WHEN("simulation is stepped until collision occurs")
			{
				INFO(sphere_info());
				step_simulation(6.3);
				step_simulation(0.1);
				INFO(sphere_info());
				THEN("spheres have rebounded off one-another")
				{
					INFO(sphere_info());
					CHECK(sphere1_body->getLinearVelocity().x() == Approx(1).epsilon(0.05));
					CHECK(sphere1_body->getLinearVelocity().y() == Approx(1).epsilon(0.05));
					CHECK(sphere1_body->getLinearVelocity().z() == 0);
					CHECK(sphere2_body->getLinearVelocity().x() == Approx(-1).epsilon(0.05));
					CHECK(sphere2_body->getLinearVelocity().y() == Approx(-1).epsilon(0.05));
					CHECK(sphere2_body->getLinearVelocity().z() == 0);
				}
			}			
		}

		
	}
}

SCENARIO("Rigid body with tile membership")
{
	GIVEN("a rigid body in a world tile")
	{
		const double sphere_mass = 1;
		const double sphere_restitution = 0.98;
		const btQuaternion sphere_orientation{0, 0, 0, 1};
		btVector3 sphere_inertia;
		auto sphere_shape = std::make_unique<btSphereShape>(1);
		sphere_shape->calculateLocalInertia(sphere_mass, sphere_inertia);

		auto sphere1_motion = std::make_unique<btDefaultMotionState>(
			btTransform(sphere_orientation, btVector3(2, 2, 0))
		);
		btRigidBody::btRigidBodyConstructionInfo sphere1_construct{
			sphere_mass, sphere1_motion.get(), sphere_shape.get(), sphere_inertia
		};
		sphere1_construct.m_restitution = sphere_restitution;

		auto sphere1_body = std::make_unique<btInfRigidBody>(sphere1_construct, btVector3{1,2,3});

		THEN("tile coordinate can be retrieved")
		{
			CHECK(sphere1_body->getTile() == btVector3{1,2,3});
		}
	}
}