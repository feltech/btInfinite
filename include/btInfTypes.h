#include <vector>

#include <LinearMath/btVector3.h>

class btInfRigidBody;

namespace btInf
{
	using TileMemberList = std::vector<btInfRigidBody*>;
	using TileMemberIdx = typename TileMemberList::size_type;
	using TileList = std::vector<TileMemberList>;
	using TileIdx = TileList::size_type;

	using TileSize = btScalar;
	using TileCoord = btVector3;

}