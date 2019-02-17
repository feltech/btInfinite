#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>

class btInfCollisionConfiguration;

/**
 *
 */
class btInfCollisionDispatcher : public btCollisionDispatcher
{
public:
	btInfCollisionDispatcher (
		btCollisionConfiguration* collisionConfiguration, const btScalar tileSize
	);

	static void defaultNearCallback(
		btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher,
		const btDispatcherInfo& dispatchInfo
	);

private:
	const btScalar m_tileSize;
};
