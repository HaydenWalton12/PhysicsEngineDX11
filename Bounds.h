#pragma once
#include "Vector.h"
class Bounds
{
public:
	Bounds();
	Bounds(const Bounds& rhs) : mins(rhs.mins), maxs(rhs.maxs)
	{

	}

	~Bounds();

	void Clear()
	{
		mins = Vec3(1e6);
		maxs = Vec3(-1e6);

	}


	bool DoesIntersect(const Bounds& rhs) const;
	
	void Expand(const Vec3 * pts , const int num);
	void Expand(const Vec3 & rhs);
	void Expand(const Bounds & rhs);

	float WidthX() const { return maxs.x - mins.x; }
	float WidthY() const { return maxs.y - mins.y; }
	float WidthZ() const { return maxs.z - mins.z; }

	Vec3 mins;
	Vec3 maxs;

private:



};

