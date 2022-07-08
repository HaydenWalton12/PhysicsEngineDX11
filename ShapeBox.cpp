#include "ShapeBox.h"

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
