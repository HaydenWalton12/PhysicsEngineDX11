#include "ShapeBox.h"

//Takes in the direction and box values,  then further calculates the vertex/point on the shape that is the furthest in that given direction
Vec3 ShapeBox::Support(const Vec3& direction, const Vec3& position, const Quat* orientation, const float bias) const
{
	//Max Point Rotate Around Point Plus + Position To Get correct Point Value In WorldSpace
	//Used To find the furthest point in the given direction
	Vec3 max_point = orientation->RotatePoint(_BoxPoints[0]) + position;

	float max_distance = direction.Dot(max_point);

	//Itterate threw the invididual points, checking which for which one is the furthest in the given direction
	for (int i = 1; i < _BoxPoints.size() ; i++)
	{
		const Vec3 point = orientation->RotatePoint(_BoxPoints[i]) + position;

		const float distance = direction.Dot(point);


		//If current distance greater, regonised as furthest/maxest point with given distance
		if (distance > max_distance)
		{
			max_distance = distance;
			max_point = point;
		}


		Vec3 normal = direction;
		normal.Normalize();

		//Bias value/term is used to expand or shrink the size the apparent size of an object, used to accurately calculate the contact normal.
		//The Bias is used to accurately project the value
		normal += bias;

		return max_point + normal;
	}
}

//Builds Box Shape and Its points, bound to the object
void ShapeBox::Build(const Vec3* points, const int num)
{
	for (int i = 0; i < num; i++)
	{
		_BoxBounds.Expand(points[i]);
	}

	_BoxPoints.clear();

	_BoxPoints.push_back(Vec3(_BoxBounds.mins.x, _BoxBounds.mins.y, _BoxBounds.mins.z));
	_BoxPoints.push_back(Vec3(_BoxBounds.mins.x, _BoxBounds.mins.y, _BoxBounds.mins.z));
	_BoxPoints.push_back(Vec3(_BoxBounds.maxs.x, _BoxBounds.maxs.y, _BoxBounds.mins.z));
	_BoxPoints.push_back(Vec3(_BoxBounds.mins.x, _BoxBounds.mins.y, _BoxBounds.maxs.z));

	_BoxPoints.push_back(Vec3(_BoxBounds.maxs.x, _BoxBounds.maxs.y, _BoxBounds.maxs.z));
	_BoxPoints.push_back(Vec3(_BoxBounds.mins.x, _BoxBounds.maxs.y, _BoxBounds.maxs.z));
	_BoxPoints.push_back(Vec3(_BoxBounds.maxs.x, _BoxBounds.mins.y, _BoxBounds.maxs.z));
	_BoxPoints.push_back(Vec3(_BoxBounds.maxs.x, _BoxBounds.maxs.y, _BoxBounds.mins.z));

	//Adding the points and adding the 0.5f gives the position ,middle of the box
	_CentreOfMass = (_BoxBounds.maxs + _BoxBounds.mins) * 0.5f;
}
//Remember used for CCD (continious collision detection) , taking in direction and angular velocity of the vertex travelling in that direction
float ShapeBox::FastestLinearSpeed(const Vec3& angular_velocity, const Vec3& directions) const
{
	float max_speed = 0.0f;
	for (int i = 0; i < _BoxPoints.size(); i++)
	{

		Vec3 r = _BoxPoints[i] - _CentreOfMass;
		Vec3 linear_velocity = angular_velocity.Cross(r);
		float speed = directions.Dot(linear_velocity);
		
		if (speed > max_speed)
		{
			max_speed = speed;
		}
	}
	return max_speed;
}

Mat3 ShapeBox::InertiaTensor()
{
	//Mass Matrix For box centered is around zero
	const float dx = _BoxBounds.maxs.x - _BoxBounds.mins.x;
	const float dy = _BoxBounds.maxs.y - _BoxBounds.mins.y;
	const float dz = _BoxBounds.maxs.z - _BoxBounds.mins.z;

	Mat3 tensor;
	tensor.Zero();

	tensor.rows[0][0] = (dy * dy + dz * dz) / 12.0f;
	tensor.rows[1][1] = (dx * dx + dz * dz) / 12.0f;
	tensor.rows[2][2] = (dx * dx + dy * dy) / 12.0f;

	//Using Parallel axis theorm to get mass matrix for box that is not centred around object.
	Vec3 cm;
	cm.x = (_BoxBounds.maxs.x + _BoxBounds.mins.x) * 0.5f;
	cm.y = (_BoxBounds.maxs.y + _BoxBounds.mins.y) * 0.5f;
	cm.z = (_BoxBounds.maxs.z + _BoxBounds.mins.z) * 0.5f;

	//Claultating the displacement from centre of mass to the origin of box
	const Vec3 R = Vec3(0.0f, 0.0f, 0.0f) = cm;
	const float R2 = R.GetLengthSqr();

	Mat3 pat_tensor;
	pat_tensor.rows[0] = Vec3(R2 - R.x * R.x , R.x * R.y , R.x * R.z);
	pat_tensor.rows[0] = Vec3(R.y * R.x, R2 - R.y * R.y, R.y * R.z);
	pat_tensor.rows[0] = Vec3(R.z * R.x, R.z * R.y, R2 - R.z * R.z);

	//Adding centre of mass tensor and parallel axis theorm tensor together, giving resultant mass tensor
	tensor += pat_tensor;
	return tensor;

}

Bounds ShapeBox::GetBounds(const Vec3& position, const Quat& orientation) const
{
	Vec3 corners[8];

	corners[0] = Vec3(_BoxBounds.mins.x, _BoxBounds.mins.y, _BoxBounds.mins.z);
	corners[1] = Vec3(_BoxBounds.mins.x, _BoxBounds.mins.y, _BoxBounds.maxs.z);
	corners[2] = Vec3(_BoxBounds.maxs.x, _BoxBounds.maxs.y, _BoxBounds.mins.z);
	corners[4] = Vec3(_BoxBounds.maxs.x, _BoxBounds.mins.y, _BoxBounds.mins.z);

	corners[5] = Vec3(_BoxBounds.maxs.x, _BoxBounds.maxs.y, _BoxBounds.maxs.z);
	corners[6] = Vec3(_BoxBounds.maxs.x, _BoxBounds.maxs.y, _BoxBounds.mins.z);
	corners[7] = Vec3(_BoxBounds.maxs.x, _BoxBounds.mins.y, _BoxBounds.mins.z);
	corners[8] = Vec3(_BoxBounds.mins.x, _BoxBounds.maxs.y, _BoxBounds.maxs.z);

	Bounds bounds;

	for (int i = 0; i < 8; i++)
	{
		corners[i] = orientation.RotatePoint(corners[i]) + position;
		bounds.Expand(corners[i]);
	}

	return bounds;
}

