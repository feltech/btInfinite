#include <algorithm>
#include <set>
#include <vector>


#include "btInfBroadphase.h"
#include "btInfRigidBody.h"

btInfBroadphase::btInfBroadphase(const btScalar tileSize)
: btSimpleBroadphase{}, m_tileSize{tileSize}
{}

void btInfBroadphase::calculateOverlappingPairs(btDispatcher* dispatcher)
{
	//first check for new overlapping pairs
	int i,j;
	if (m_numHandles >= 0)
	{
		std::set<void*> overlapping = {};

		int new_largest_index = -1;
		for (i = 0; i <= m_LastHandleIndex; i++)
		{
			btSimpleBroadphaseProxy* proxy0 = &m_pHandles[i];
			if(!proxy0->m_clientObject)
				continue;
			new_largest_index = i;

			for (j = i + 1; j <= m_LastHandleIndex; j++)
			{
				btSimpleBroadphaseProxy* proxy1 = &m_pHandles[j];
				btAssert(proxy0 != proxy1);
				if(!proxy1->m_clientObject)
					continue;

				if (aabbOverlap(proxy0, proxy1))
				{
					overlapping.insert(proxy0->m_clientObject);
					overlapping.insert(proxy1->m_clientObject);
					if (!m_pairCache->findPair(proxy0,proxy1))
					{
						m_pairCache->addOverlappingPair(proxy0, proxy1);
					}
				}
				else
				{
					if (!m_pairCache->hasDeferredRemoval())
					{
						if ( m_pairCache->findPair(proxy0, proxy1))
						{
							m_pairCache->removeOverlappingPair(proxy0, proxy1, dispatcher);
						}
					}
				}
			}
		}

		m_LastHandleIndex = new_largest_index;
		for (i = 0; i <= m_LastHandleIndex; i++)
		{
			btSimpleBroadphaseProxy& proxy = m_pHandles[i];
			btInfRigidBody* body = static_cast<btInfRigidBody*>(proxy.m_clientObject);
			if (overlapping.count(body))
				continue;

			body->m_refTile = btInfRigidBody::NO_REF;
			const btVector3& origin = body->getWorldTransform().getOrigin();

			const btScalar tileRadius = m_tileSize / 2;
			const btVector3 tileDelta{
				std::trunc(origin.x() / tileRadius),
				std::trunc(origin.y() / tileRadius),
				std::trunc(origin.z() / tileRadius)
			};
			static const btVector3 zero{0, 0, 0};
			if (tileDelta != zero)
			{
				body->m_tileCoord += tileDelta;
				body->getWorldTransform().setOrigin(origin - tileDelta * m_tileSize);
			}

		}

		if (m_ownsPairCache && m_pairCache->hasDeferredRemoval())
		{
			btBroadphasePairArray&	overlappingPairArray = m_pairCache->getOverlappingPairArray();

			//perform a sort, to find duplicates and to sort 'invalid' pairs to the end
			overlappingPairArray.quickSort(btBroadphasePairSortPredicate());

			overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
			m_invalidPair = 0;

			btBroadphasePair previousPair;
			previousPair.m_pProxy0 = 0;
			previousPair.m_pProxy1 = 0;
			previousPair.m_algorithm = 0;


			for (i = 0; i < overlappingPairArray.size(); i++)
			{

				btBroadphasePair& pair = overlappingPairArray[i];

				bool isDuplicate = (pair == previousPair);

				previousPair = pair;

				bool needsRemoval = false;

				if (!isDuplicate)
				{
					bool hasOverlap = testAabbOverlap(pair.m_pProxy0, pair.m_pProxy1);

					if (hasOverlap)
					{
						needsRemoval = false;//callback->processOverlap(pair);
					} else
					{
						needsRemoval = true;
					}
				} else
				{
					//remove duplicate
					needsRemoval = true;
					//should have no algorithm
					btAssert(!pair.m_algorithm);
				}

				if (needsRemoval)
				{
					m_pairCache->cleanOverlappingPair(pair,dispatcher);

					//		m_overlappingPairArray.swap(i,m_overlappingPairArray.size()-1);
					//		m_overlappingPairArray.pop_back();
					pair.m_pProxy0 = 0;
					pair.m_pProxy1 = 0;
					m_invalidPair++;
				}

			}

			///if you don't like to skip the invalid pairs in the array, execute following code:
#define CLEAN_INVALID_PAIRS 1
#ifdef CLEAN_INVALID_PAIRS

			//perform a sort, to sort 'invalid' pairs to the end
			overlappingPairArray.quickSort(btBroadphasePairSortPredicate());

			overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
			m_invalidPair = 0;
#endif//CLEAN_INVALID_PAIRS

		}
	}
}


bool btInfBroadphase::testAabbOverlap(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1)
{
	btSimpleBroadphaseProxy* p0 = getSimpleProxyFromProxy(proxy0);
	btSimpleBroadphaseProxy* p1 = getSimpleProxyFromProxy(proxy1);
	return aabbOverlap(p0, p1);
}

bool btInfBroadphase::aabbOverlap(
	btSimpleBroadphaseProxy* proxy0, btSimpleBroadphaseProxy* proxy1
) {
	const btVector3& tileCoord0 = static_cast<btInfRigidBody*>(proxy0->m_clientObject)->m_tileCoord;
	const btVector3& tileCoord1 = static_cast<btInfRigidBody*>(proxy1->m_clientObject)->m_tileCoord;

	const btVector3& tileOffset1 = m_tileSize * (tileCoord1 - tileCoord0);
	const btVector3& aabbMin1 = proxy1->m_aabbMin + tileOffset1;
	const btVector3& aabbMax1 = proxy1->m_aabbMax + tileOffset1;

	return proxy0->m_aabbMin[0] <= aabbMax1[0] &&
		aabbMin1[0] <= proxy0->m_aabbMax[0] &&
		proxy0->m_aabbMin[1] <= aabbMax1[1] &&
		aabbMin1[1] <= proxy0->m_aabbMax[1] &&
		proxy0->m_aabbMin[2] <= aabbMax1[2] &&
		aabbMin1[2] <= proxy0->m_aabbMax[2];

}

