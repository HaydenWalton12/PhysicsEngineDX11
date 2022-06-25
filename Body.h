#pragma once
#include "Shape.h"

//A physics simulation , or any applied physics within a game is a collection of bodies that essentially colide and get manipulated by environmental ingame factors.
//Each body has a position within space , represented utilising vectors in cartesian coordinates (since this is a 3D space, we utilise vec3 class for all
//positional related manipulations/changes).

//The bodies implied have to be derived from a shape , a shape can be considiered a bounding volume for an object itself , or an individual represnetivie object
//we utilise a variety of shapes within simulations, said shapes all vary with the mathematics to determine physics properties and simulated effects.However all
//principles cross over between varying shapes.


//The Engine Will Handle All Forces As Direct Impusles Within The Engine 
// Impulses are defined as an integral of a specfic force acting over a period of time.
// We apply forces this way since this is contextual to how acting forces are applied in the real world.
// No Forces (with the exception of gravity) are applied at a constant , all forces are applied
// contextually upon the environment/conditions evolving around objects with mass.

//Impulse is just the change of momentum



class Body
{
public:

	//Hold Where it is (position) in scene/world space
	Vec3 _Position;
	//Determines the speed of object throughout world space.Determines direction of position of movement too
	Vec3 _LinearVelocity;
	//Determines Speed OF how quick the orientation of object rotates 
	Vec3 _AngularVelocity;
	//Will store orientation 
	Quat _Orientation;
	//Holds what type the shape of the invidual body is.
	Shape* _Shape;

	Vec3 _Gravity = Vec3(0.0f , -9.8f , 0.0f);
	float _Friction;

	//Inverse Mass Is used over sotring typical mass since we need a way to describe objects considered having infinite mass , objects in the real world
	//dont have infinite mass ,however we treat objects as such in comparison to other objects, since the mass could be so insignificant to how
	//it is effected by forces, we disregard the mass. For our case, we want our plane or terrain of the world to consider having infinite mass.
	//This is why we treat forces as an impulse, since depending on the inverse mass value itself, we may not want that force to applied to achieve semi-realistic
	//results.
	//if invmass = 0 , that means the mass is infinite and forces wont apply to object
	float _InvMass;
	float _Elasicity;

	void Update(float dt_sec);
	void ApplyImpulse(Vec3 impulse_point, Vec3 impulse);
	void AddImpulseLinear(const Vec3& impulse);
	void AddImpulseAngular(const Vec3& impulse);
	Mat3 GetInverseInertiaTensorBodySpace();
	Mat3 GetInverseInertiaTensorWorldSpace();
	
	
	//When bodies collide or traverse , we want to collect the information about where in world space that collision or action is happening.
	//Remember world space is the main space for all objects converted from local space. To be integrated into one plane where they all hold their values in position
	//within

	//To get the point of where the body collided, we will want a function that can transform worldspace to local space, along with a function that transforms 
	//local space to world space. Incremental for gathering and representing bodies.

	//We also need to include body space, body space is the space inate the physics where it is centered around a bodies centre of mass, different from model space
	//where it centers around shapes geometric origin , or world space where its a collection of geomtry with positions. Body space
	//goes one step further into a bodies centre of mass.

	//These functions will become incremental in physics calculations.


	//Getting Mass Is important for the calculation of determining forces and the factors applied from such conditions. With mass we can calculate weight.
	Vec3 GetCenterOfMassWorldSpace() const;
	Vec3 GetCenterOfMassModelSpace() const;
	Vec3 WorldSpaceToBodySpace(const Vec3& world_pt) const;
	Vec3 BodySpaceToWorldSpace(const Vec3& world_pt) const;

};
