#define CATCH_CONFIG_MAIN
#include <algorithm>

#include "catch.hpp"
#include <btBulletDynamicsCommon.h>
#include "btInfCollisionDispatcher.h"
#include "btInfRigidBody.h"
#include "btInfBroadphase.h"


std::ostream& operator <<( std::ostream& os, const btVector3& p ) {
	os << "(" << p.x() << ", " << p.y() << ", " << p.z() << ")";
	return os;
}

SCENARIO("Sphere-sphere collision")
{
	const btScalar tile_size = 10;
	auto broadphase = std::make_unique<btInfBroadphase>(tile_size);
	auto collision_config = std::make_unique<btDefaultCollisionConfiguration>();
	auto dispatcher = std::make_unique<btInfCollisionDispatcher>(collision_config.get(), tile_size);
	auto solver = std::make_unique<btSequentialImpulseConstraintSolver>();
	auto world = std::make_unique<btDiscreteDynamicsWorld>(
		dispatcher.get(), broadphase.get(), solver.get(), collision_config.get()
	);

	world->setGravity(btVector3(0,0,0));

	double sphere_mass = 1;
	double sphere_restitution = 0.97;
	btQuaternion sphere_orientation{0, 0, 0, 1};
	btVector3 sphere_inertia;
	auto sphere_shape = std::make_unique<btSphereShape>(0.5);
	sphere_shape->calculateLocalInertia(sphere_mass, sphere_inertia);

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

	auto sphere_info = [&](const auto& sphere_body) {
		std::stringstream ss;
		ss << "pos(" << sphere_body->getCenterOfMassPosition().x() <<
			", " << sphere_body->getCenterOfMassPosition().y() <<
			", " << sphere_body->getCenterOfMassPosition().z() << ") " <<
			" vel(" << sphere_body->getLinearVelocity().x() <<
			", " << sphere_body->getLinearVelocity().y() <<
			", " << sphere_body->getLinearVelocity().z() << ") " <<
			" tile(" << sphere_body->m_tileCoord.x() <<
			", " << sphere_body->m_tileCoord.y() <<
			", " << sphere_body->m_tileCoord.z() << ") " <<
			" world(" << (
				sphere_body->m_tileCoord.x() * tile_size +
				sphere_body->getCenterOfMassPosition().x()
			 ) << ", " << (
				sphere_body->m_tileCoord.y() * tile_size +
				sphere_body->getCenterOfMassPosition().y()
			 ) << ", " << (
				sphere_body->m_tileCoord.z() * tile_size +
				sphere_body->getCenterOfMassPosition().z()
			 ) << ") ";

		return ss.str();
	};


	GIVEN("Two spheres in the same tile")
	{
		std::unique_ptr<btInfRigidBody> sphere1_body;
		std::unique_ptr<btInfRigidBody> sphere2_body;

		auto spheres_info = [&]() {
			std::stringstream ss;
			ss << "sphere1 " << sphere_info(sphere1_body) << std::endl << "sphere2 " <<
			sphere_info(sphere2_body) << std::endl;
			return ss.str();
		};

		sphere1_body = create_sphere(btVector3{1,2,3}, btVector3{2, 2, 0});
		sphere2_body = create_sphere(btVector3{1,2,3}, btVector3{-2, -2, 0});

		WHEN("spheres are sent flying toward one-another")
		{
			sphere1_body->applyCentralImpulse(btVector3(-1, -1, 0));
			sphere2_body->applyCentralImpulse(btVector3(1, 1, 0));

			AND_WHEN("simulation is stepped until just before the collision")
			{
				INFO(spheres_info());
				step_simulation(1.0);
				INFO(spheres_info());
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
					INFO(spheres_info());
					step_simulation(0.7);

					THEN("spheres have rebounded off one-another")
					{
						INFO(spheres_info());
						CHECK(sphere1_body->getLinearVelocity().x() == Approx(1).margin(0.05));
						CHECK(sphere1_body->getLinearVelocity().y() == Approx(1).margin(0.05));
						CHECK(sphere1_body->getLinearVelocity().z() == 0);
						CHECK(sphere2_body->getLinearVelocity().x() == Approx(-1).margin(0.05));
						CHECK(sphere2_body->getLinearVelocity().y() == Approx(-1).margin(0.05));
						CHECK(sphere2_body->getLinearVelocity().z() == 0);
					}
				}
			}
		}
	}
	GIVEN("Two spheres in neighbouring tiles")
	{
		std::unique_ptr<btInfRigidBody> sphere1_body;
		std::unique_ptr<btInfRigidBody> sphere2_body;
		sphere1_body = create_sphere(btVector3{1,1,0}, btVector3{2, 2, 0});
		sphere2_body = create_sphere(btVector3{0,0,0}, btVector3{-2, -2, 0});

		auto spheres_info = [&]() {
			std::stringstream ss;
			ss << "sphere1 " << sphere_info(sphere1_body) << std::endl << "sphere2 " <<
			sphere_info(sphere2_body) << std::endl;
			return ss.str();
		};

		WHEN("spheres are sent flying toward one-another")
		{
			sphere1_body->applyCentralImpulse(btVector3(-1, -1, 0));
			sphere2_body->applyCentralImpulse(btVector3(1, 1, 0));
			INFO(spheres_info());

			AND_WHEN("simulation is stepped a little")
			{
				step_simulation(1.0/60);
				INFO(spheres_info());

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
				step_simulation(1.7);
				INFO(spheres_info());
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
				step_simulation(6.7);
				INFO(spheres_info());
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
				step_simulation(6.9);
				INFO(spheres_info());
				THEN("spheres have rebounded off one-another")
				{
					CHECK(sphere1_body->getLinearVelocity().x() == Approx(1).margin(0.05));
					CHECK(sphere1_body->getLinearVelocity().y() == Approx(1).margin(0.05));
					CHECK(sphere1_body->getLinearVelocity().z() == 0);
					CHECK(sphere2_body->getLinearVelocity().x() == Approx(-1).margin(0.05));
					CHECK(sphere2_body->getLinearVelocity().y() == Approx(-1).margin(0.05));
					CHECK(sphere2_body->getLinearVelocity().z() == 0);
				}
			}
		}

	} // End: Two spheres in neighbouring tiles

	GIVEN("Two pairs of spheres in distant tiles colliding like Newtons cradle")
	{

		std::unique_ptr<btInfRigidBody> sphere11_body, sphere12_body, sphere21_body, sphere22_body;
		sphere11_body = create_sphere(btVector3{-2,0,0}, btVector3{-2, 0, 0});
		sphere12_body = create_sphere(btVector3{-2,0,0}, btVector3{0, 0, 0});
		sphere21_body = create_sphere(btVector3{2,0,0}, btVector3{2, 0, 0});
		sphere22_body = create_sphere(btVector3{2,0,0}, btVector3{0, 0, 0});

		sphere11_body->applyCentralImpulse(btVector3(1, 0, 0));
		sphere21_body->applyCentralImpulse(btVector3(-1, 0, 0));

		auto spheres_info = [&]() {
			std::stringstream ss;
			ss << "sphere11=" << sphere_info(sphere11_body) << std::endl <<
				"sphere12=" << sphere_info(sphere12_body) << std::endl <<
				"sphere21=" << sphere_info(sphere21_body) << std::endl <<
				"sphere22=" << sphere_info(sphere22_body) << std::endl << std::endl;
			return ss.str();
		};

		WHEN("pairs collide")
		{
			step_simulation(1.05);
			INFO(spheres_info());

			THEN("they've chosen a suitable reference tile for collision calculations")
			{
				CHECK(sphere11_body->m_refTile == btVector3(-2, 0, 0));
				CHECK(sphere12_body->m_refTile == btVector3(-2, 0, 0));
				CHECK(sphere21_body->m_refTile == btVector3(2, 0, 0));
				CHECK(sphere22_body->m_refTile == btVector3(2, 0, 0));
			}
			THEN("they have expected velocities")
			{
				CHECK(sphere11_body->getLinearVelocity().x() == Approx(0).margin(0.1));
				CHECK(sphere12_body->getLinearVelocity().x() == Approx(1).margin(0.1));
				CHECK(sphere21_body->getLinearVelocity().x() == Approx(0).margin(0.1));
				CHECK(sphere22_body->getLinearVelocity().x() == Approx(-1).margin(0.1));
			}
		}

		WHEN("pairs have rebounded far away from one another")
		{
			step_simulation(6.05);
			THEN("broadphase no longer flags potential overlap")
			{
				CHECK(
					broadphase->getOverlappingPairCache()->getNumOverlappingPairs()
					== 0
				);
			}

			THEN("tile coords have been updated")
			{
				CHECK(sphere12_body->m_tileCoord == btVector3(-1, 0, 0));
				CHECK(sphere22_body->m_tileCoord == btVector3(1, 0, 0));
			}
		}

		WHEN("rebounded spheres meet in the middle")
		{
			step_simulation(6.05);
			step_simulation(13.2);
			std::stringstream ss;
			ss << "sphere11=" << sphere_info(sphere11_body) << std::endl <<
				"sphere12=" << sphere_info(sphere12_body) << std::endl <<
				"sphere21=" << sphere_info(sphere21_body) << std::endl <<
				"sphere22=" << sphere_info(sphere22_body) << std::endl << std::endl;
			INFO(ss.str());

			THEN("broadphase flags potential overlap")
			{
				CHECK(
					broadphase->getOverlappingPairCache()->getNumOverlappingPairs()
					== 1
				);

				std::vector<void*> clientObjects{
					broadphase->getOverlappingPairCache()->
						getOverlappingPairArray()[0].m_pProxy0->m_clientObject,
					broadphase->getOverlappingPairCache()->
						getOverlappingPairArray()[0].m_pProxy1->m_clientObject
				};

				CHECK_THAT(
					clientObjects, Catch::Matchers::Contains(std::vector<void*>{
						sphere12_body.get(), sphere22_body.get()
					})
				);
			}
			THEN("tile coords have been updated")
			{
				CHECK(sphere12_body->m_tileCoord == btVector3(0, 0, 0));
				CHECK(sphere22_body->m_tileCoord == btVector3(0, 0, 0));
			}
			THEN("chosen reference tile is between the two")
			{
				CHECK(sphere12_body->m_refTile == btVector3(0, 0, 0));
				CHECK(sphere22_body->m_refTile == btVector3(0, 0, 0));
			}
		}
	}
}