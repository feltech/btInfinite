#include <limits>

#include "btInf/RigidBody.h"

namespace btInf
{

RigidBody::RigidBody(
	const btRigidBodyConstructionInfo& constructionInfo, const btVector3& tileCoord
) : btRigidBody{constructionInfo}, m_tileCoord{tileCoord},
	m_idxTileMember{std::numeric_limits<btInf::TileMemberIdx>::max()}
{}

}
