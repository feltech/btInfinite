#include <limits>

#include <LinearMath/btVector3.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

#include "btInfTypes.h"

class btInfRigidBody : public btRigidBody
{
public:
	btInf::TileCoord	m_tileCoord;
	btInf::TileCoord	m_refTileCoord;
	btInf::TileMemberIdx	m_idxTileMember;

	static const btVector3 NO_REF;

public:

	btInfRigidBody(
		const btRigidBodyConstructionInfo& constructionInfo, const btVector3& tileCoord
	);
};




