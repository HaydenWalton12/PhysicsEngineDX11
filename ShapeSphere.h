#pragma once
#include "Shape.h"


class ShapeSphere : public Shape
{
public:

	ShapeType _ShapeType;
	float _Radius;


	//Used To Initialise The Object

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

	//The Support Function allows us to find the point on a convex shaoe that is the furthest in a particular direction.
	//For Example- Lets say our cube is moving -x , on the cube , this function will return a point on the cube that is the most furthest in that direction, anypoint on a surface
	//of the cube
	Vec3 Support(const Vec3& direction, const Vec3& position, const Quat* orientation, const float bias) const override;


	Mat3 InertiaTensor() override;
	Bounds GetBounds(const Vec3& pos, const Quat& orient)  const override;
	Bounds GetBounds() const override;
	ShapeType GetType() const override;

};