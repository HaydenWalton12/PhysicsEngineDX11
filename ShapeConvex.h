#pragma once 
#include "Shape.h"
struct tri_t
{
	int a;
	int b;
	int c;
};
struct edge_t
{
	int a;
	int b;

	//used to assign edges with their values
	bool operator == (const edge_t& rhs) const
	{
		return ((a == rhs.a && b == rhs.b) || (a == rhs.b && b == rhs.a));
	}


};

class ShapeConvex : public Shape
{

public:

	ShapeType _ShapeType;
	std::vector<Vec3> _BoxPoints;
	Bounds _BoxBounds;


	//Used To Initialise The Object

	ShapeConvex(const Vec3* points, const int num, Surface surface, RenderCommands* render, TextureComponent* tex, XMFLOAT3 translate, XMFLOAT3 scale, XMFLOAT3 rotate)
	{
		Build(points, num);
		_CentreOfMass.Zero();

		//Sets Up THe Object Bound To THis Shape , Replacing having to individualy define a shape in a scene class to the same length as this
		_Object = new Object(render, L"Floor.dds", tex, "Cube.Obj");
		_Object->SetSurface(surface.Ambient, surface.Diffuse, surface.Specular, surface.SpecularPower);

		_Object->SetVertexShader(L"DX11 Framework.fx");
		_Object->SetPixelShader(L"DX11 Framework.fx");

		_Object->SetTransformation(translate, scale, rotate);

		//Allows us to get the assigned type of Shape
		_ShapeType = SHAPE_CONVEX;
	}

	//The Support Function allows us to find the point on a convex shaoe that is the furthest in a particular direction.
	//For Example- Lets say our cube is moving -x , on the cube , this function will return a point on the cube that is the most furthest in that direction, anypoint on a surface
	//of the cube
	Vec3 Support(const Vec3& direction, const Vec3& position, const Quat* orientation, const float bias) const override;

	void Build(const Vec3* points, const int num) override;



	float FastestLinearSpeed(const Vec3& angular_velocity, const Vec3& directions) const override;

	//Box Has Different InertiaTensor , Remember The Inertia Tensor Is Considered The Mass Matrix , Used TO Distribute Mass Correctly Throughout The OBject
	Mat3 InertiaTensor() override;
	Bounds GetBounds(const Vec3& pos, const Quat& orient)  const override;
	Bounds GetBounds() const override;
	ShapeType GetType() const override;


	void AddPoint(std::vector<Vec3>& hull_points, std::vector<tri_t>& hull_triangles, const Vec3& point);
	int FindPointInFurthestDirection(const Vec3* point, const int num, const Vec3& direction);
	float DistanceFromLine(const Vec3& a, const Vec3& b, const Vec3& point);
	Vec3 FindPointFurthestFromLine(const Vec3* points, const int num, const Vec3& point_a, const Vec3& point_b);
	float DistanceFromTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& point);
	Vec3 FindPointFurthestFromTriangle(const Vec3* points, const int num, const Vec3& point_a, const Vec3& point_b, const Vec3 point_c);
	void BuildTetrahedron(const Vec3* verticies, const int num, std::vector<Vec3>& hull_points, std::vector<tri_t>& hull_triangles);
	void ExpandConvexHull(std::vector<Vec3>& hull_points, std::vector<tri_t>& hull_triangles, const std::vector<Vec3> verticies);
	void RemoveInternalPoints(const std::vector<Vec3>& hull_points, const std::vector<tri_t>& hull_triangles, std::vector<Vec3>& check_points);
	bool IsEdgeUnique(const std::vector<tri_t>& triangles, const std::vector<int>& facing_triangles, const int ignore_triangles
		, const edge_t& edge);
	void RemoveUnreferencedVerticies(std::vector<Vec3>& hull_points, std::vector<tri_t>& hull_triangles);
	void BuildConvexHull(const std::vector<Vec3>& verticies, std::vector<Vec3>& hull_points, std::vector<tri_t>& hull_triangles);
	bool IsExternal(const std::vector<Vec3>& points, const std::vector<tri_t>& triangles, const Vec3& point);
	Vec3 CalculateCentreOfMass(const std::vector<Vec3>& points, const std::vector<tri_t>& triangles);
	Mat3 CalculateInertiaTensor(const std::vector<Vec3>& points, const std::vector<tri_t>& triangles, const Vec3& cm);



};

