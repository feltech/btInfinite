#ifndef btInfTypes_h
#define btInfTypes_h
#include <vector>

#include <LinearMath/btVector3.h>

namespace btInf
{
	class RigidBody;
	using TileMemberList = std::vector<RigidBody*>;
	using TileMemberIdx = typename TileMemberList::size_type;
	using TileList = std::vector<TileMemberList>;
	using TileIdx = TileList::size_type;

	using TileSize = btScalar;
	using TileCoord = btVector3;
}
#endif
