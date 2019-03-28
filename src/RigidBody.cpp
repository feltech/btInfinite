#include "btInf/RigidBody.h"

namespace btInf
{

RigidBody::RigidBody(
	const btRigidBodyConstructionInfo& constructionInfo, const btVector3& tileCoord
) : btRigidBody{constructionInfo}, m_tileCoord{tileCoord}, m_refTileCoord{RigidBody::NO_REF}
{}

const btVector3 RigidBody::NO_REF{
	std::numeric_limits<float>::max(),
	std::numeric_limits<float>::max(),
	std::numeric_limits<float>::max()
};

}
