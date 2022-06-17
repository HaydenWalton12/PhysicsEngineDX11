#pragma once
#include "Shape.h"
class Body
{
public:

	

	Vec3 _Position;
	Vec3 _LinearVelocity;
	Vec3 _AngularVelocity;

	Quat _Orientation;
	Shape* _Shape;

	float _Friction;
	float _InvMass;
	float _Elasicity;

	void Update(float dt_sec);

	void ApplyImpulse(Vec3 impulse_point, Vec3 impulse);
	void AddImpulseLinear(const Vec3& impulse);
	void AddImpulseAngular(const Vec3& impulse);
	Mat3 GetInverseInertiaTensorBodySpace();
	Mat3 GetInverseInertiaTensorWorldSpace();
	Vec3 GetCenterOfMassWorldSpace() const;
	Vec3 GetCenterOfMassModelSpace() const;
	Vec3 WorldSpaceToBodySpace(const Vec3& world_pt) const;
	Vec3 BodySpaceToWorldSpace(const Vec3& world_pt) const;

};
