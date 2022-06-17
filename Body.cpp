#include "Body.h"

void Body::ApplyImpulse(Vec3 impulse_point, Vec3 impulse)
{
	
	if (0.0f == _InvMass){return;}

	AddImpulseLinear(impulse);

	Vec3 position = GetCenterOfMassWorldSpace();
	Vec3 r = impulse_point - position;
	Vec3 dL = r.Cross(impulse);

	AddImpulseAngular(dL);
}

Mat3 Body::GetInverseInertiaTensorBodySpace()
{
	Mat3 inertia_tensor = _Shape->InertiaTensor();
	Mat3 inverse_inertia_tensor = inertia_tensor.Inverse() * _InvMass;
	return inverse_inertia_tensor;
}

Mat3 Body::GetInverseInertiaTensorWorldSpace()
{
	Mat3 inertia_tensor = _Shape->InertiaTensor();
	Mat3 inverse_inertia_tensor = inertia_tensor.Inverse() * _InvMass;
	Mat3 orient = _Orientation.ToMat3() * _InvMass;
	inverse_inertia_tensor = orient * inverse_inertia_tensor * orient.Transpose();

	return inverse_inertia_tensor;
}

void Body::AddImpulseLinear(const Vec3& impulse)
{
	if (0.0f == _InvMass){return;}
	_LinearVelocity += impulse * _InvMass;
}

void Body::AddImpulseAngular(const Vec3& impulse)
{
	if (0.0f == _InvMass){return;}

	_AngularVelocity += GetInverseInertiaTensorWorldSpace() * impulse;

	const float max_angular_speed = 30.0f;

	if (_AngularVelocity.GetLengthSqr() > max_angular_speed * max_angular_speed)
	{
		_AngularVelocity.Normalize();
		_AngularVelocity *= max_angular_speed;
	}
}

void Body::Update(float dt_sec)
{
	_Position += _LinearVelocity * dt_sec;
	
	Vec3 position_cm = GetCenterOfMassWorldSpace();
	Vec3 cm_to_pos = _Position - position_cm;

	Mat3 orientation = _Orientation.ToMat3();
	Mat3 inertia_tensor = orientation * _Shape->InertiaTensor() * orientation.Transpose();
	Vec3 alpha = inertia_tensor.Inverse() * (_AngularVelocity.Cross(inertia_tensor * _AngularVelocity ));
	Vec3 angle = alpha * dt_sec;

	Vec3 dAngle = _AngularVelocity * dt_sec;
	Quat dq = Quat(dAngle, dAngle.GetMagnitude());
	_Orientation = dq * _Orientation;
	_Orientation.Normalize();

	_Position = position_cm + dq.RotatePoint(cm_to_pos);

}

Vec3 Body::GetCenterOfMassWorldSpace() const
{
	const Vec3 centre_of_mass = _Shape->GetCentreOfMass();
	const Vec3 position = _Position + _Orientation.RotatePoint(centre_of_mass);
	return position;
}

Vec3 Body::GetCenterOfMassModelSpace() const
{
	const Vec3 centre_of_mass = _Shape->GetCentreOfMass();
	return centre_of_mass;
}

Vec3 Body::WorldSpaceToBodySpace(const Vec3& world_pt) const
{
	Vec3 temp = world_pt - GetCenterOfMassWorldSpace();
	Quat inverse_orientation = _Orientation.Inverse();
	Vec3 body_space = inverse_orientation.RotatePoint(temp);

	return body_space;
}

Vec3 Body::BodySpaceToWorldSpace(const Vec3& world_pt) const
{
	Vec3 world_space = GetCenterOfMassWorldSpace() + _Orientation.RotatePoint(world_pt);
	return world_space;
}
