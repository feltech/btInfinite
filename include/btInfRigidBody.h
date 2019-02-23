#include <limits>

#include <LinearMath/btVector3.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

class btInfRigidBody : public btRigidBody
{
public:
	btVector3	m_tileCoord;
	btVector3	m_refTile;
	static const btVector3 NO_REF;
	static constexpr float TILE_SIZE = 10000.0f;

public:

	btInfRigidBody(
		const btRigidBodyConstructionInfo& constructionInfo, const btVector3& tileCoord
	);
};




