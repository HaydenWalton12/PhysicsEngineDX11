#pragma once
#include "Body.h"
#include "ShapeSphere.h"
//Collisions

//Struct Used For Interpentration , utilising points of contacts between two bodies 
struct Contact
{
	Vec3 ptOnA_WorldSpace;
	Vec3 ptOnB_WorldSpace;

	Vec3 ptOnA_LocalSpace;
	Vec3 ptOnB_LocalSpace;


	//In World Space Coordinates , Normalizzed Direction From Point A-B Either World Or Local Space
	Vec3 Normal;

	//Value positive when non-pentrating , negative when penetratinhg
	float _SeperationDistance;
	float _TimeOfImpact;

	Body* _BodyA;
	Body* _BodyB;

};


class Collision
{
public:
	Collision();
	~Collision();

	static bool Intersect(Body* A, Body* B, float dt, Contact& contact);
	static bool SphereSphereDynamic(ShapeSphere* sphere_a, ShapeSphere* sphere_b, Vec3& pos_a, Vec3& pos_b, Vec3& vel_a, Vec3& vel_b, float dt, Vec3& pt_On_A, Vec3& pt_On_B, float& toi);
	static bool RaySphere(Vec3 ray_start, Vec3& ray_direction, Vec3 sphere_centre, float sphere_radius, float& t1, float& t2);
private:

};

