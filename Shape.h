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
	};
	virtual ShapeType GetType() const = 0;
	Object* _Object;
};

class ShapeSphere : public Shape
{
public:
	float _Radius;

	ShapeSphere(float radius , Surface surface , RenderCommands* render , TextureComponent* tex , XMFLOAT3 translate , XMFLOAT3 scale , XMFLOAT3 rotate)
	{
		_Radius = radius;
		_Object = new Object(render, L"floor.dds", tex, "cube.Obj");
		_Object->SetSurface(surface.Ambient, surface.Diffuse, surface.Specular, surface.SpecularPower);
		_Object->SetVertexShader(L"DX11 Framework.fx");
		_Object->SetPixelShader(L"DX11 Framework.fx");

		_Object->SetTransformation(translate , scale ,rotate);
	
	}
	
	//Allows us to get the assigned type of Shape
	ShapeType GetType() const override { return SHAPE_SPHERE; };

	//This Object is needed to define the object within the framework
	




};

class Body
{
public:

	Vec3 _Position;
	Quat _Orientation;
	Shape* _Shape;

};

