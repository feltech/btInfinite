#include <BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h>
#include <btInfRigidBody.h>
#include "btInfCollisionDispatcher.h"


btInfCollisionDispatcher::btInfCollisionDispatcher (
	btCollisionConfiguration* collisionConfiguration, const btScalar tileSize
) : btCollisionDispatcher(collisionConfiguration), m_tileSize{tileSize}
{
	this->setNearCallback(btInfCollisionDispatcher::defaultNearCallback);
}


void btInfCollisionDispatcher::defaultNearCallback(
	btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher,
	const btDispatcherInfo& dispatchInfo
) {
	btInfRigidBody* colObj0 = (btInfRigidBody*)collisionPair.m_pProxy0->m_clientObject;
	btInfRigidBody* colObj1 = (btInfRigidBody*)collisionPair.m_pProxy1->m_clientObject;

	if (dispatcher.needsCollision(colObj0,colObj1))
	{
		btVector3 refTile;
		if (colObj0->m_refTileCoord != btInfRigidBody::NO_REF) {
			refTile = colObj0->m_refTileCoord;
			colObj1->m_refTileCoord = refTile;
		} else if (colObj1->m_refTileCoord != btInfRigidBody::NO_REF) {
			refTile = colObj1->m_refTileCoord;
			colObj0->m_refTileCoord = refTile;
		} else {
			refTile = colObj0->m_tileCoord;
			colObj0->m_refTileCoord = refTile;
			colObj1->m_refTileCoord = refTile;
		}
		const btScalar tileSize = static_cast<btInfCollisionDispatcher&>(dispatcher).m_tileSize;
		const btVector3 tileDelta0 = tileSize * (colObj0->m_tileCoord - refTile);
		const btVector3 tileDelta1 = tileSize * (colObj1->m_tileCoord - refTile);

		btTransform transform0{colObj0->getWorldTransform()};
		transform0.setOrigin(transform0.getOrigin() + tileDelta0);
		btTransform transform1{colObj1->getWorldTransform()};
		transform1.setOrigin(transform1.getOrigin() + tileDelta1);

		btCollisionObjectWrapper obj0Wrap(
			0, colObj0->getCollisionShape(), colObj0, transform0, -1, -1
		);
		btCollisionObjectWrapper obj1Wrap(
			0, colObj1->getCollisionShape(), colObj1, transform1, -1, -1
		);

		//dispatcher will keep algorithms persistent in the collision pair
		if (!collisionPair.m_algorithm)
		{
			collisionPair.m_algorithm = dispatcher.findAlgorithm(
				&obj0Wrap, &obj1Wrap, 0, BT_CONTACT_POINT_ALGORITHMS
			);
		}

		if (collisionPair.m_algorithm)
		{
			btManifoldResult contactPointResult(&obj0Wrap,&obj1Wrap);

			if (dispatchInfo.m_dispatchFunc == btDispatcherInfo::DISPATCH_DISCRETE)
			{
				//discrete collision detection query
				collisionPair.m_algorithm->processCollision(
					&obj0Wrap, &obj1Wrap, dispatchInfo, &contactPointResult
				);
			}
			else
			{
				//continuous collision detection query, time of impact (toi)
				btScalar toi = collisionPair.m_algorithm->calculateTimeOfImpact(
					colObj0, colObj1, dispatchInfo, &contactPointResult
				);
				if (dispatchInfo.m_timeOfImpact > toi)
					dispatchInfo.m_timeOfImpact = toi;
			}
		}
	}

}
