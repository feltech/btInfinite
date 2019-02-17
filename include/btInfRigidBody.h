#include <limits>

#include <LinearMath/btVector3.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

class btInfRigidBody : public btRigidBody
{
	btVector3	m_tileCoord;
	btVector3	m_refTile;
public:
	static const btVector3 NO_REF;
	static constexpr float TILE_SIZE = 10000.0f;

public:

	btInfRigidBody(
		const btRigidBodyConstructionInfo& constructionInfo, const btVector3& tileCoord
	);

	const btVector3& getTile() const;
	const btVector3& getRefTile() const;
	void setRefTile(const btVector3& refCoord);
};




