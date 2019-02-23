#include <btInfRigidBody.h>


btInfRigidBody::btInfRigidBody(
	const btRigidBodyConstructionInfo& constructionInfo, const btVector3& tileCoord
) : btRigidBody{constructionInfo}, m_tileCoord{tileCoord}, m_refTile{btInfRigidBody::NO_REF}
{}

const btVector3 btInfRigidBody::NO_REF{
	std::numeric_limits<float>::max(),
	std::numeric_limits<float>::max(),
	std::numeric_limits<float>::max()
};