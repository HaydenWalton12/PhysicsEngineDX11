#include "ShapeSphere.h"

Shape::ShapeType ShapeSphere::GetType() const
{
	return _ShapeType;
}

Mat3 ShapeSphere::InertiaTensor()
{
	Mat3 Tensor;
	Tensor.Zero();

	Tensor.rows[0][0] = 2.0f * _Radius * _Radius / 5.0f;
	Tensor.rows[1][1] = 2.0f * _Radius * _Radius / 5.0f;
	Tensor.rows[2][2] = 2.0f * _Radius * _Radius / 5.0f;
	return Tensor;
}

Bounds ShapeSphere::GetBounds(const Vec3& pos, const Quat& orient) const
{
	Bounds tmp;
	tmp.mins = Vec3(-_Radius) + pos;
	tmp.maxs = Vec3(_Radius) + pos;
	return tmp;
}

Bounds ShapeSphere::GetBounds() const
{
	Bounds tmp;
	tmp.mins = Vec3(-_Radius);
	tmp.maxs = Vec3(_Radius);
	return tmp;
}
