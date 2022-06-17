#pragma once
#include "Shape.h"


class ShapeSphere : public Shape
{
public:

	ShapeType _ShapeType;
	float _Radius;

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
	}

	Mat3 InertiaTensor() override;
	Bounds GetBounds(const Vec3& pos, const Quat& orient)  const override;
	Bounds GetBounds() const override;
	ShapeType GetType() const override;
};