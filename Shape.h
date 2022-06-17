#pragma once
#include "Object.h"
#include "Vector.h"
#include "Quanternion.h"
#include "Bounds.h"
class Shape
{
public:
	enum ShapeType
	{
		SHAPE_SPHERE,
		SHAPE_CUBE,
		SHAPE_PLANE,
	};

	virtual ShapeType GetType() const = 0;

	ShapeType type;


	virtual Bounds GetBounds(const Vec3& pos, const Quat& orient)  const = 0;
	virtual Bounds GetBounds() const = 0;

	virtual Vec3 GetCentreOfMass() const { return _CentreOfMass; }

	//Inate Objects Will Be Passed Down To Child CLasses, Security Access Modifer Enabling Inheritance Principle
	Object* _Object;

	//Will Be Explained Later - However an integral value to the Physics System
	Vec3 _CentreOfMass;

	virtual Mat3 InertiaTensor()
	{
		Mat3 o;
		return o;
	}

};



