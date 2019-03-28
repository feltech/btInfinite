#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>

namespace btInf
{
/**
 *
 */
class CollisionDispatcher : public btCollisionDispatcher
{
public:
	CollisionDispatcher (
		btCollisionConfiguration* collisionConfiguration, const btScalar tileSize
	);

	static void defaultNearCallback(
		btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher,
		const btDispatcherInfo& dispatchInfo
	);

private:
	const btScalar m_tileSize;
};
}