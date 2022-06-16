#pragma once
#include "Object.h"
#include "Vector.h"
#include "Quanternion.h"
class Shape
{
public:
	enum ShapeType
	{
		SHAPE_SPHERE,
		SHAPE_CUBE,
		SHAPE_PLANE,
	};
	virtual ShapeType GetType()
	{
		return type;
	}

	ShapeType type;

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

class ShapeSphere : public Shape
{
public:

	ShapeType _ShapeType;

	float _Radius;
	ShapeType GetType() override
	{
		return _ShapeType;
	}
	ShapeSphere(float radius, Surface surface, RenderCommands* render, TextureComponent* tex, XMFLOAT3 translate, XMFLOAT3 scale, XMFLOAT3 rotate)
	{
		_Radius = radius;
		_CentreOfMass.Zero();

		//Sets Up THe Object Bound To THis Shape , Replacing having to individualy define a shape in a scene class to the same length as this
		_Object = new Object(render, L"Floor.dds", tex, "sphere.Obj");
		_Object->SetSurface(surface.Ambient, surface.Diffuse, surface.Specular, surface.SpecularPower);

		_Object->SetVertexShader(L"DX11 Framework.fx");
		_Object->SetPixelShader(L"DX11 Framework.fx");

		_Object->SetTransformation(translate, scale, rotate);

		//Allows us to get the assigned type of Shape
	  _ShapeType = SHAPE_SPHERE;


		//This Object is needed to define the object within the framework
	}

	Mat3 InertiaTensor() override
	{
		Mat3 Tensor;
		Tensor.Zero();

		Tensor.rows[0][0] = 2.0f * _Radius * _Radius / 5.0f;
		Tensor.rows[1][1] = 2.0f * _Radius * _Radius / 5.0f;
		Tensor.rows[2][2] = 2.0f * _Radius * _Radius / 5.0f;
		return Tensor;
	}
};

class Body
{
public:

	Vec3 _Position;
	Quat _Orientation;
	Vec3 _AngularVelocity;
	Shape* _Shape;
	float _Friction;
	float _InvMass;
	float _Elasicity;
	Vec3 _LinearVelocity;

	void ApplyImpulse(Vec3 impulse_point , Vec3 impulse)
	{
		
		if (0.0f == _InvMass)
		{
			return;
		}

		AddImpulseLinear(impulse);

		Vec3 position = GetCenterOfMassWorldSpace();
		Vec3 r = impulse_point - position;
		Vec3 dL = r.Cross(impulse);

		AddImpulseAngular(dL);
	}

	Mat3 GetInverseInertiaTensorBodySpace()
	{
		Mat3 inertiaTensor = _Shape->InertiaTensor();
		Mat3 invInertiaTensor = inertiaTensor.Inverse() * _InvMass;
		return invInertiaTensor;
	}
	Mat3 GetInverseInertiaTensorWorldSpace()
	{
		Mat3 inertiaTensor = _Shape->InertiaTensor();
		Mat3 invInertiaTensor = inertiaTensor.Inverse() * _InvMass;
		Mat3 orient = _Orientation.ToMat3() * _InvMass;
		invInertiaTensor = orient * invInertiaTensor * orient.Transpose();

		return invInertiaTensor;
	}
	void AddImpulseLinear(const Vec3& impulse)
	{
		if (0.0f == _InvMass)
		{
			return;
		}
		//p = mv
		//dp = m dv = j
		//=> dv = j / m

		_LinearVelocity += impulse * _InvMass;


	}
	void AddImpulseAngular(const Vec3& impulse)
	{
		if (0.0f == _InvMass)
		{
			return;
		}

		_AngularVelocity += GetInverseInertiaTensorWorldSpace() * impulse;

		const float max_angular_speed = 30.0f;

		if (_AngularVelocity.GetLengthSqr() > max_angular_speed * max_angular_speed)
		{
			_AngularVelocity.Normalize();
			_AngularVelocity *= max_angular_speed;
		}
	}

	void Update(float dt_sec)
	{
		Vec3 position_cm = GetCenterOfMassWorldSpace();
		Vec3 cm_to_pos = _Position - position_cm;

		Mat3 orientation = _Orientation.ToMat3();
		Mat3 inertia_tensor = orientation * _Shape->InertiaTensor() * orientation.Transpose();

		Vec3 alpha = inertia_tensor.Inverse() * (_AngularVelocity.Cross(inertia_tensor * _AngularVelocity += alpha * dt_sec));

		Vec3 angle = _AngularVelocity * dt_sec;
		Quat dq = Quat(angle, angle.GetMagnitude());

		_Orientation = dq * _Orientation;
		_Orientation.Normalize();

		_Position = position_cm + dq.RotatePoint(cm_to_pos);
		_Position += _LinearVelocity * dt_sec;

	}

	//Applying The World Posstion  and Orietnation to the centre of mass will update the position relative to world space.
	Vec3 GetCenterOfMassWorldSpace() const
	{
		const Vec3 centre_of_mass = _Shape->GetCentreOfMass();
		const Vec3 position = _Position + _Orientation.RotatePoint(centre_of_mass);
	
		return position;
	
	}
	Vec3 GetCenterOfMassModelSpace() const
	{
		const Vec3 centre_of_mass = _Shape->GetCentreOfMass();
		
		return centre_of_mass;
	
	}
	Vec3 WorldSpaceToBodySpace(const Vec3& world_pt) const
	{
		Vec3 temp = world_pt - GetCenterOfMassWorldSpace();
		Quat inverse_orientation = _Orientation.Inverse();
		Vec3 body_space = inverse_orientation.RotatePoint(temp);

		return body_space;
	}
	Vec3 BodySpaceToWorldSpace(const Vec3& world_pt) const
	{
		Vec3 world_space = GetCenterOfMassWorldSpace() + _Orientation.RotatePoint(world_pt);
	
		return world_space;
	
	}

};
