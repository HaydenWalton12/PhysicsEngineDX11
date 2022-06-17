#pragma once
#include "Shape.h"

#include "Body.h"

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

