#include "BroadPhase.h"

struct psuedoBody
{
	int id;
	float value;
	bool is_min;

};

void BuildPairs(std::vector <CollisionPair>& collision_pairs, const psuedoBody* sorted_bodies, const int num)
{
	collision_pairs.clear();

	//Since Bodies Are sorted, create collision pairs
	for (int i = 0; i < num * 2; i++)
	{
		const psuedoBody& a = sorted_bodies[i];
		if (!a.is_min)
		{
			continue;
		}

		CollisionPair pair;
		pair.a = a.id;

		for (int j = i + 1; j < num * 2; j++)
		{
			const psuedoBody& b = sorted_bodies[j];

			if (b.id == a.id)
			{
				break;

			}
			if (!b.is_min)
			{
				continue;
			}
			pair.b = b.id;
			collision_pairs.push_back(pair);
		}
	}
}


int CompareSap(const void* a, const void* b)
{

	const psuedoBody * ea = (const psuedoBody*)a;
	const psuedoBody * eb = (const psuedoBody*)b;


	if (ea->value < eb->value)
	{
		return -1;
	}
	return 1;
}

void SortBodiesBounds(const Body* bodies, const int num, psuedoBody* sorted_array, const float dt_sec)
{
	Vec3 axis = Vec3(1.0f, 1.0f, 1.0f);
	axis.Normalize();

	for (int i = 0; i < num; i++)
	{
		const Body& body = bodies[i];

		Bounds bounds = body._Shape->GetBounds(body._Position, body._Orientation);

		bounds.Expand(bounds.mins + body._LinearVelocity * dt_sec);
		bounds.Expand(bounds.maxs + body._LinearVelocity * dt_sec);

		const float epsilon = 0.01f;

		bounds.Expand(bounds.mins + Vec3(-1.0f, -1.0f, -1.0f) * epsilon);
		bounds.Expand(bounds.mins + Vec3(1.0f, 1.0f, 1.0f) * epsilon);


		sorted_array[i * 2 + 0].id = i;
		sorted_array[i * 2 + 0].value = axis.Dot(bounds.mins);
		sorted_array[i * 2 + 0].is_min = true;


		sorted_array[i * 2 + 1].id = i;
		sorted_array[i * 2 + 1].value = axis.Dot(bounds.maxs);
		sorted_array[i * 2 + 1].is_min = false;

	}

	qsort(sorted_array, num * 2, sizeof(psuedoBody), CompareSap);

}
void SweepAndPrune1D(const Body* bodies, const int num, std::vector<CollisionPair>& final_pairs, const float dt_sec)
{
	psuedoBody* sorted_bodies = (psuedoBody*)alloca(sizeof(psuedoBody) * num * 2);
	SortBodiesBounds(bodies, num, sorted_bodies, dt_sec);
	BuildPairs(final_pairs, sorted_bodies, num);


}

void BroadPhase(const Body* bodies, const int num, std::vector<CollisionPair>& final_pairs, const float dt_sec)
{
	final_pairs.clear();

	SweepAndPrune1D(bodies, num, final_pairs, dt_sec);
}