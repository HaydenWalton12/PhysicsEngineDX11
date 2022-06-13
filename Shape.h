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
	

	virtual Vec3 GetCentreOfMass() const { return _CentreOfMass; }

	//Inate Objects Will Be Passed Down To Child CLasses, Security Access Modifer Enabling Inheritance Principle
	Object* _Object;


	//Will Be Explained Later - However an integral value to the Physics System
	Vec3 _CentreOfMass;

};

class ShapeSphere : public Shape
{
public:


	float _Radius;

	ShapeSphere(float radius , Surface surface , RenderCommands* render , TextureComponent* tex , XMFLOAT3 translate , XMFLOAT3 scale , XMFLOAT3 rotate)
	{
		_Radius = radius;
		_CentreOfMass.Zero();

		//Sets Up THe Object Bound To THis Shape , Replacing having to individualy define a shape in a scene class to the same length as this
		_Object = new Object(render, L"Floor.dds", tex, "cube.Obj");
		_Object->SetSurface(surface.Ambient, surface.Diffuse, surface.Specular, surface.SpecularPower);
		
		_Object->SetVertexShader(L"DX11 Framework.fx");
		_Object->SetPixelShader(L"DX11 Framework.fx");

		_Object->SetTransformation(translate , scale ,rotate);
	}
	
	//Allows us to get the assigned type of Shape
	ShapeType _ShapeType = SHAPE_SPHERE;

	//This Object is needed to define the object within the framework
	
};

class ShapePlane : public Shape
{
public:



   ShapePlane(Surface surface, RenderCommands* render, TextureComponent* tex, XMFLOAT3 translate, XMFLOAT3 scale, XMFLOAT3 rotate)
	{

		_CentreOfMass.Zero();

		//Sets Up THe Object Bound To THis Shape , Replacing having to individualy define a shape in a scene class to the same length as this
		_Object = new Object(render, L"Floor.dds", tex, "cube.Obj");
		_Object->SetSurface(surface.Ambient, surface.Diffuse, surface.Specular, surface.SpecularPower);
		_Object->SetVertexShader(L"DX11 Framework.fx");
		_Object->SetPixelShader(L"DX11 Framework.fx");

		_Object->SetTransformation(translate, scale, rotate);
	}

	//Allows us to get the assigned type of Shape
	 ShapeType _ShapeType = SHAPE_PLANE;

	//This Object is needed to define the object within the framework





};
class Body
{
public:

	Vec3 _Position;
	Quat _Orientation;
	Shape* _Shape;

	float _InvMass;

	//Position of an body Change When They have velocity , alligned with a equation 
	Vec3 _LinearVelocity;

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
