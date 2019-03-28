#include <BulletCollision/BroadphaseCollision/btSimpleBroadphase.h>

#include "btInf/Types.h"

namespace btInf
{
class Broadphase : public btSimpleBroadphase
{
public:
	Broadphase(const btInf::TileSize tileSize, btInf::TileList& tiles);

	void calculateOverlappingPairs(btDispatcher* dispatcher);
	bool aabbOverlap(
		btSimpleBroadphaseProxy* proxy0, btSimpleBroadphaseProxy* proxy1
	);
	bool testAabbOverlap(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1);

	virtual btBroadphaseProxy* createProxy(
		const btVector3& aabbMin, const btVector3& aabbMax, int shapeType, void* userPtr,
		int collisionFilterGroup, int collisionFilterMask, btDispatcher* dispatcher
	);

private:
	btInf::TileIdx tileHash(const btInf::TileCoord& tileCoord);
	const btScalar m_tileSize;

	btInf::TileList& m_tiles;
};
}
