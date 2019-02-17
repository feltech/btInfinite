#include <BulletCollision/BroadphaseCollision/btSimpleBroadphase.h>

class btInfBroadphase : public btSimpleBroadphase
{
public:
	btInfBroadphase(const btScalar tileSize);

	void calculateOverlappingPairs(btDispatcher* dispatcher);
	bool aabbOverlap(
		btSimpleBroadphaseProxy* proxy0, btSimpleBroadphaseProxy* proxy1
	);
	bool testAabbOverlap(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1);


private:
	const btScalar m_tileSize;
};