#pragma once
#include "Body.h"

struct CollisionPair
{
	int a, b;

	bool operator == (const CollisionPair& rhs) const
	{
		return ( ( (a == rhs.a) && (b == rhs.b) ) || ( (a == rhs.b) && (b == rhs.a) ) );

	}

	bool operator !=(const CollisionPair& rhs) const
	{
		return !(*this == rhs);
	}
};

void BroadPhase(const Body* bodies, const int num, std::vector<CollisionPair>& final_pairs, const float dt_sec);
