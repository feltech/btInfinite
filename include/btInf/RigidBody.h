#include <limits>

#include <LinearMath/btVector3.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

#include "btInf/Types.h"

namespace btInf
{
class RigidBody : public btRigidBody
{
public:
	btInf::TileCoord	m_tileCoord;
	btInf::TileCoord	m_refTileCoord;
	btInf::TileMemberIdx	m_idxTileMember;

	static const btVector3 NO_REF;

public:

	RigidBody(
		const btRigidBodyConstructionInfo& constructionInfo, const btVector3& tileCoord
	);
};
}