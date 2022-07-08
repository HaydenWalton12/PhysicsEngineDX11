#pragma once
#include "Object.h"
#include "Vector.h"
#include "Quanternion.h"
#include "Bounds.h"


//Parent class hold all reference to the key invidual segments of each class. Each child shape class will hold attritbutes inate to the shape.


class Shape
{
public:
	//Defined Shape Types
	enum ShapeType
	{
		SHAPE_SPHERE,
		SHAPE_BOX,
		SHAPE_CONVEX,
	};



	virtual void Build(const Vec3* pts, const int num) {}


	//Both Support And FastestLinearSpeed Functions will be used in assitance of calculating the collision 
	//detection between general convex shapes and for the continuous collision detection , remember continious collision drtection
	//is the relative principle we use to determine collision of narrowphased objects, to prevent teleporting or unpicked collision movement between frames where
	//collision tests are not being applied , this is due to how the update loop is developed.
	virtual Vec3 Support(const Vec3& direction, const Vec3& position, const Quat* orientation, const float bias) const = 0;


	//Used for collision detection , used for all objects and assists in collision detection for "long" objects, since a long object that is rotating might hit other objects howeer may 
	// have zero linear velocity , it may only have angular velocity that is causing it to rotate, we can consider this as a effect for assisting in objects that are
	//colliding not because of linear velocity but rotating/angular velocity, hence why we pass the angular velocity and the direction , we can get the direction from
	//the support function.
	virtual float FastestLinearSpeed(const Vec3& angular_velocity, const Vec3& directions) const { return 0.0f; }




	virtual ShapeType GetType() const = 0;

	//Holds self type inheritated to fill value in child 
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



