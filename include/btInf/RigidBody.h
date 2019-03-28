#include <limits>
#include <optional>

#include <LinearMath/btVector3.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

#include "btInf/Types.h"

namespace btInf
{
class RigidBody : public btRigidBody
{
public:
	btInf::TileCoord	m_tileCoord;
	std::optional<btInf::TileCoord>	m_refTileCoord;
	btInf::TileMemberIdx	m_idxTileMember;

public:

	RigidBody(
		const btRigidBodyConstructionInfo& constructionInfo, const btVector3& tileCoord
	);
};
}
