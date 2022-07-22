#include "ShapeConvex.h"

//Takes in the direction and box values,  then further calculates the vertex/point on the shape that is the furthest in that given direction
Vec3 ShapeConvex::Support(const Vec3& direction, const Vec3& position, const Quat* orientation, const float bias) const
{
	//Max Point Rotate Around Point Plus + Position To Get correct Point Value In WorldSpace
	//Used To find the furthest point in the given direction
	Vec3 max_point = orientation->RotatePoint(_BoxPoints[0]) + position;

	float max_distance = direction.Dot(max_point);

	//Itterate threw the invididual points, checking which for which one is the furthest in the given direction
	for (int i = 1; i < _BoxPoints.size(); i++)
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
void ShapeConvex::Build(const Vec3* points, const int num)
{
	_BoxPoints.clear();
	_BoxPoints.reserve(num);

	//Add points
	for (int i = 0; i < num; i++)
	{
		_BoxPoints.push_back(points[i]);
	}

	//Expand into a convex hull
	std::vector<Vec3> hull_points;
	std::vector<tri_t> hull_triangles;

	BuildConvexHull(_BoxPoints, hull_points, hull_triangles);
	_BoxPoints = hull_points;

	//Expand Bounds

	_BoxBounds.Clear();
	_BoxBounds.Expand(_BoxPoints.data(), _BoxPoints.size());


	_CentreOfMass = CalculateCentreOfMass(hull_points, hull_triangles);
	InertiaTensor() = CalculateInertiaTensor(hull_points, hull_triangles, _CentreOfMass);

}
//Remember used for CCD (continious collision detection) , taking in direction and angular velocity of the vertex travelling in that direction
float ShapeConvex::FastestLinearSpeed(const Vec3& angular_velocity, const Vec3& directions) const
{
	//Works the same as ShapeBox iteration of function , since the physics applies to both shape types
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

Mat3 ShapeConvex::InertiaTensor()
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
	pat_tensor.rows[0] = Vec3(R2 - R.x * R.x, R.x * R.y, R.x * R.z);
	pat_tensor.rows[0] = Vec3(R.y * R.x, R2 - R.y * R.y, R.y * R.z);
	pat_tensor.rows[0] = Vec3(R.z * R.x, R.z * R.y, R2 - R.z * R.z);

	//Adding centre of mass tensor and parallel axis theorm tensor together, giving resultant mass tensor
	tensor += pat_tensor;
	return tensor;

}


//Getting bounds is used to define the boundaries for shape,, applying it to corners while adding the orientation to accurately define these points
Bounds ShapeConvex::GetBounds(const Vec3& position, const Quat& orientation) const
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

void  ShapeConvex::AddPoint(std::vector<Vec3>& hull_points, std::vector<tri_t>& hull_triangles, const Vec3& point)
{

	//Replacing old triangles to build new triangles with the newly changed and optimised point lists

	//Used to find all triangles that face at this point
	std::vector<int> facing_triangles;
	for (int i = (int)hull_triangles.size() - 1; i >= 0; i--)
	{
		const tri_t& tri = hull_triangles[i];

		const Vec3& a = hull_points[tri.a];
		const Vec3& b = hull_points[tri.b];
		const Vec3& c = hull_points[tri.c];

		const float distance = DistanceFromTriangle(a, b, c, point);
		if (distance > 0.0f)
		{
			facing_triangles.push_back(i);
		}
	}

	//Finding all edges that are unique to the triangles, these will be edges that will form newly optimise convex and triangles

	std::vector<edge_t> unique_edges;
	for (int i = 0; i < facing_triangles.size(); i++)
	{
		const int triangle_id = facing_triangles[i];
		const tri_t& triangle = hull_triangles[triangle_id];

		//Edges enact as the vertex points that will be used to fill index list of triangle order to create new triangle
		edge_t edges[3];

		edges[0].a = triangle.a;
		edges[0].b = triangle.b;

		edges[1].a = triangle.b;
		edges[1].b = triangle.c;

		edges[2].a = triangle.c;
		edges[2].b = triangle.a;

		for (int e = 0; e < 3; e++)
		{
			//Checks for each edge being unique
			if (IsEdgeUnique(hull_triangles, facing_triangles, triangle_id, edges[e]))
			{
				//add new edge if it unique with all points and values given.
				unique_edges.push_back(edges[e]);
			}


		}
	}

	//Need to remove old triangles/values from lists 
	for (int i = 0; i < facing_triangles.size(); i++)
	{
		//Appropriate method of erase since old triangles given in said values, all new edges/values assigned 
		//to "unique_edges"
		hull_triangles.erase(hull_triangles.begin() + facing_triangles[i]);


	}
	//Add new points
	hull_points.push_back(point);

	const int new_point_id = (int)hull_points.size() - 1;

	//Adding triangles for each unique edge
	for (int i = 0; i < unique_edges.size(); i++)
	{
		const edge_t& edge = unique_edges[i];

		tri_t triangle;

		triangle.a = edge.a;
		triangle.b = edge.b;
		triangle.c = new_point_id;

		hull_triangles.push_back(triangle);
	}

}


//We Further Need to construct the inertia tensor , furthe we need to be certain we are only storing the points of the convex hull
//these points are further the surface as applied to all objects.

//This said we jave to slowly build up connections of the points , if points are inside the convex hull , we remove them , we can achieve this by
// building a simplex, tehen expanding it out to include all points outside it.

//All functions below are resonsible for this process

//Used to find the point that is the furthest in a given direction (this is essentially the support function we have created for the box) however works slightly differently
int ShapeConvex::FindPointInFurthestDirection(const Vec3* point, const int num, const Vec3& direction)
{
	int max_id = 0;

	//Max distance from give value from the given point

	float max_distance = direction.Dot(point[0]);
	for (int i = 0; i < num; i++)
	{
		float distance = direction.Dot(point[0]);
		//Replace new max distance point and change the ID to relevant point given in the current iteration of the loop
		if (distance > max_distance)
		{
			max_distance = distance;
				max_id = i;
		}

	}
		
	return max_id;


}


//Used to find another point that is furthest in the opposite direction of the point , this is typically given from the max point found from the function above
float ShapeConvex::DistanceFromLine(const Vec3& a, const Vec3& b, const Vec3& point)
{
	Vec3 ab = b - a;
	ab.Normalize();

	Vec3 ray = point - a;
	
	Vec3 projection = ab * ray.Dot(ab);
	Vec3 perpindicular = ray - projection;

	return perpindicular.GetMagnitude();
}


//this function finds the point furthest from the axis of pooints formed from the previous two functions

Vec3 ShapeConvex::FindPointFurthestFromLine(const Vec3* points, const int num, const Vec3& point_a, const Vec3& point_b)
{

	int max_id = 0;

	//Find Distance / Max distance using function above , this will the highest/max distance for now to compare underneath with all other given points of the convex
	float max_distance = DistanceFromLine(point_a, point_b, points[0]);

	for (int i = 0; i < num; i++)
	{

		float distance = DistanceFromLine(point_a, point_b, points[i]);

		
		if (distance > max_distance)
		{
			max_distance = distance;
			max_id = i;
		}
	}
	//Returns point with the furthest distance with the given two points , points taken from previous two functions processing points
	return points[max_id];
}



//We then find the point that is furthest from the plane formed from previous processed points
//We collate individual points, use this to form a triangle of points, we then find the facing normal direction to see how far the triangle of points is
//This will be used later to build an array of index points now precalculated to create a convex shape
float ShapeConvex::DistanceFromTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3 & point)
{

	Vec3 ab = b - a;
	Vec3 ac = c - a;

	//Get the outfacing vector between these vertex points
	Vec3 normal = ab.Cross(ac);
	normal.Normalize();

	Vec3 ray = point - a;
	float distance = ray.Dot(normal);

	return distance;

}

//Passing the individual vertex points that compose a triangle , we pass in the additional created convex points to then calculate the max distance of points that when collated
//used in accompany above to find which points are furthest from triangles being collated from the points
Vec3 ShapeConvex::FindPointFurthestFromTriangle(const Vec3* points, const int num, const Vec3& point_a, const Vec3& point_b, const Vec3 point_c)
{

	int max_id = 0;

	//Find initial maximum distance 
	float max_distance = DistanceFromTriangle(point_a, point_b, point_c, points[0]);


	//Iterate current findings with aforementioned points given
	for (int i = 0; i < num; i++)
	{
		float distance = DistanceFromTriangle(point_a, point_b, point_c, points[i]);

		///If Doesnt Work Use this
		//if (distance * distance  > max_distance * max_distance)
		if (distance > max_distance)
		{


			max_distance = distance;

			max_id = i;
		}
	
		return points[max_id];
	}
}

//With all given and said, we can now build the convex shape using the primtive points and traingle we p[re-calculated , this will be possible with the utility of all the given functions
//above ,allowing 
void ShapeConvex::BuildTetrahedron(const Vec3 * verticies , const int num , std::vector<Vec3> & hull_points , std::vector<tri_t>& hull_triangles )
{
	hull_points.clear();
	hull_triangles.clear();

	Vec3 points[4];

	int idx = FindPointInFurthestDirection(verticies, num, Vec3(1.0f, 0.0f, 0.0f));
	points[0] = verticies[idx];
	idx = FindPointInFurthestDirection(verticies, num, points[0] * -1.0f);
 
	points[1] = verticies[idx];
	points[2] = FindPointFurthestFromLine(verticies, num, points[0], points[1]);
	points[3] = FindPointFurthestFromTriangle(verticies, num, points[0], points[1], points[2]);

	//Important for making sure ordering is for culling order of drawing indicies to be correct
	float distance = DistanceFromTriangle(points[0], points[1], points[2], points[3]);

	if (distance > 0.0f)
	{
		std::swap(points[0], points[1]);

	}

	//Collate Points to build tetrahedron
	hull_points.push_back(points[0]);
	hull_points.push_back(points[1]);
	hull_points.push_back(points[2]);
	hull_points.push_back(points[3]);

	tri_t tri;
	tri.a = 0;
	tri.b = 1;
	tri.c = 2;
	hull_triangles.push_back(tri);

	tri.a = 0;
	tri.b = 2;
	tri.c = 3;

	hull_triangles.push_back(tri);

	tri.a = 2;
	tri.b = 1;
	tri.c = 3;
	hull_triangles.push_back(tri);

	tri.a = 1;
	tri.b = 0;
	tri.c = 3;
	hull_triangles.push_back(tri);

}

//We now need to further check the end of the function that each faces is draw order is counter clock wise, we uuse normals to calulcate each face 
//we need to create an outile to perform to expand the tetradron to the created convex hull

void ShapeConvex::ExpandConvexHull(std::vector<Vec3>& hull_points, std::vector<tri_t>& hull_triangles, const std::vector<Vec3> verticies)
{

	std::vector<Vec3> external_verticies = verticies;

	RemoveInternalPoints(hull_points, hull_triangles, external_verticies);

	while (external_verticies.size() > 0)
	{
		int point_ID = FindPointInFurthestDirection(external_verticies.data(), (int)external_verticies.size(), external_verticies[0]);

		Vec3 point = external_verticies[point_ID];

		//Remove this element
		external_verticies.erase(external_verticies.begin() + point_ID);

		AddPoint(hull_points, hull_triangles, point);

		RemoveInternalPoints(hull_points, hull_triangles, external_verticies);


	}

	RemoveUnreferencedVerticies(hull_points, hull_triangles);


}

//RemovalInteralPoints is a brute force function , iterating threw a list to check if any points are inside the current convex hull
//if they are , we remove points from the list

void ShapeConvex::RemoveInternalPoints(const std::vector<Vec3>& hull_points, const std::vector<tri_t>& hull_triangles, std::vector<Vec3>& check_points)
{
	for (int i = 0; i < check_points.size() ; i++)
	{
		const Vec3& point = check_points[i];

		bool is_external = false;

		for (int j = 0; j < hull_triangles.size(); j++)
		{

			//Establish Tri Point Connections
			const tri_t& tri = hull_triangles[j];
			const Vec3& a = hull_points[tri.a];
			const Vec3& b = hull_points[tri.b];
			const Vec3& c = hull_points[tri.c];

			//Point is in front of any triangle then it becomes external
			float distance = DistanceFromTriangle(a , b , c ,point);

			//If the point is in front of any traingle ,it is external , distance , since remember we use the "DistanceFromTriangle"
			//to calculate distance at a point with given triangle points , if larger , it means the point within the list of convex points is 
			//external
			if (distance > 0.0f)
			{
				is_external = true;
				break;
			}
		}

		//If not external , then it is also inside the convex shape and needs to be removed
		if (!is_external)
		{
			check_points.erase(check_points.begin() + i);
			//Removes point from iteration list
			i--;
		}	
	}
	//We can further remove any points that are a little close to the given hull points
	//Checking remaining points prior to seeing whether they are external or not
	for (int i = 0; i < check_points.size(); i++)
	{

		const Vec3& point = check_points[i];

		bool too_close = false;

		for (int j = 0; j < hull_points.size(); j++)
		{

			Vec3 hull_point = hull_points[j];

			//Get Given Current Vertex Position difference
			Vec3 point_check = hull_point - point;

			if (point_check.GetLengthSqr() < 0.01f * 0.01f ) // Measuring Instance of 1cm, 1cm too close
			{
				too_close = true;
				break;
			}
		}

		//Remove vertex point that is too close
			
		if (too_close)
		{
			check_points.erase(check_points.begin() + i);
			//Again , deduct point from iteration within loop
			i--;
		}

	}
}

//Add Point function , we will loop over all the triangles and determine if the point is on either positivie side of the traingle
//if it is , delete the triangles, once done we will find the remaining edges and build new traingles , this is done to simplify the amount
//of tris on a given convex , to reduce and optimise per tri/point calculations on the triangle , as if a point/tri can be made redudant remove it

bool ShapeConvex::IsEdgeUnique(const std::vector<tri_t>& triangles, const std::vector<int>& facing_triangles, const int ignore_triangles
	, const edge_t& edge)
{
	for (int i = 0; i < facing_triangles.size(); i++)
	{
		const int triangle_id = facing_triangles[i];

		//Triangle to ignore will be ignored 
		if (ignore_triangles == triangle_id)
		{
			continue;
		}

		const tri_t& triangle = triangles[triangle_id];

		edge_t edges[3];

		edges[0].a = triangle.a;
		edges[0].b = triangle.b;

		edges[1].a = triangle.a;
		edges[1].b = triangle.b;

		edges[2].a = triangle.a;
		edges[2].b = triangle.b;

		for (int e = 0; e < 3; e++)
		{

			if (edge == edges[e])
			{
				return false;
			}
		}
	}

	return true;

}



//We still need to remove the points that create the triangle, this function does that , we loop over each point 
//checking if it referenced in new triangle , then simply if isnt, remove it
void ShapeConvex::RemoveUnreferencedVerticies(std::vector<Vec3>& hull_points, std::vector<tri_t>& hull_triangles)
{
	for (int i = 0; i < hull_points.size(); i++)
	{
		bool is_used = false;

		//If the current hull point iteration instance within the conditional is true(meaning the hull point
		//iteration correctly adds too the current triangle referenced in nested loop" it means this currently triangle is used. 
		
		for (int j = 0; j < hull_triangles.size(); j++)
		{
			const tri_t& triangle = hull_triangles[j];

			if (triangle.a == i || triangle.b == i || triangle.c == i)
			{
				is_used == true;
				break;
			}
		}
		
		//Will continue to next iteration instead of checking below if it isnt referenced
		if (is_used == true)
		{
			continue;
		}
		//if the current triangle referenced in nested loop is greater than current reference 
		//triangle, this means the triangle is not used, we remove it as seen in conditions
		for (int j = 0; j < hull_triangles.size(); j++)
		{
			tri_t& triangle = hull_triangles[j];

			if (triangle.a > i)
			{
				triangle.a--;
			}
			if (triangle.b > i)
			{
				triangle.b--;
			}
			if (triangle.c > i)
			{
				triangle.c--;
			}
		}
		//Erease point as mentioned
		hull_points.erase(hull_points.begin() + i);
		i--;
	}
}

//Correclty builds convex hull
void ShapeConvex::BuildConvexHull(const std::vector<Vec3>& verticies, std::vector<Vec3>& hull_points, std::vector<tri_t>& hull_triangles)
{
	if (verticies.size() < 4)
	{
		return;
	}

	//Build tetrahedron
	BuildTetrahedron(verticies.data(), (int)verticies.size(), hull_points, hull_triangles);
	ExpandConvexHull(hull_points, hull_triangles, verticies);
}


//Still need to calculate the massMatrix (inertia tensor) , to do this ,we will decompose the entire convex hull into individual
//chunks of tetrahedrons , calculating the inertia tensor for each chunk , then total the result together.


bool ShapeConvex::IsExternal(const std::vector<Vec3>& points, const std::vector<tri_t>& triangles, const Vec3& point)
{
	bool is_external = false;

	for (int t = 0; t < triangles.size(); t++)
	{
		//Assign traingle to said value
		const tri_t & triangle = triangles[t];

		const Vec3& a = points[triangle.a];
		const Vec3& b = points[triangle.b];
		const Vec3& c = points[triangle.c];
	

		//If the points is bfront of the triangles, we can identify as the point itself is external

		float distance = DistanceFromTriangle(a, b, c, point);

		//External factor will be any value above 0, > 0 = external point as distance determines this
		if (distance > 0.0f)
		{
			is_external = true;
			break;
		}
	}

	return is_external;
}

//Calculating Centre Of Mass

Vec3 ShapeConvex::CalculateCentreOfMass(const std::vector<Vec3> & points , const std::vector<tri_t> & triangles)
{
	const int num_samples = 100;

	Bounds bounds;
	//Expand to accomidate each point 
	bounds.Expand(points.data(), points.size());

	Vec3 cm(0.0f);

	//Calulating bounds for individual axes for points
	const float dx = bounds.WidthX() / (float)num_samples;
	const float dy = bounds.WidthY() / (float)num_samples;
	const float dz = bounds.WidthZ() / (float)num_samples;

	int sample_count = 0;
	for (float x = bounds.mins.x; x < bounds.maxs.x ; x += dx)
	{
		for (float y = bounds.mins.y; y < bounds.maxs.y; x += dy)
		{
			for (float z = bounds.mins.z; z < bounds.maxs.z; z += dz)
			{
				Vec3 point(x, y, z);
				if (IsExternal(points, triangles, point))
				{
					continue;
				}
				cm += point;
				sample_count++;
			}
		}
	}
	cm /= (float)sample_count;
	return cm;
}

//Calculating mass matrix/intertia tensor
Mat3 ShapeConvex::CalculateInertiaTensor(const std::vector<Vec3>& points, const std::vector<tri_t>& triangles, const Vec3& cm)
{
	const int num_samples = 100;

	Bounds bounds;
	bounds.Expand(points.data(), (int)points.size());

	Mat3 tensor;
	tensor.Zero();

	//Calulating bounds for individual axes for points
	const float dx = bounds.WidthX() / (float)num_samples;
	const float dy = bounds.WidthY() / (float)num_samples;
	const float dz = bounds.WidthZ() / (float)num_samples;

	int sample_count = 0;
	for (float x = bounds.mins.x; x < bounds.maxs.x; x += dx)
	{for (float y = bounds.mins.y; y < bounds.maxs.y; x += dy)
	{for (float z = bounds.mins.z; z < bounds.maxs.z; z += dz)
		{
			Vec3 point(x, y, z);
			if (IsExternal(points, triangles, point))
			{
				continue;
			}
			
			//Gets point relative to centre of mass
			point -= cm;

			//Distributing calculations among mass matrix
			tensor.rows[0][0] += point.y * point.y + point.z * point.z;
			tensor.rows[1][1] += point.z * point.z + point.x * point.x;
			tensor.rows[2][2] += point.x * point.x + point.y * point.y;

			tensor.rows[0][1] += -1.0f * point.x * point.y;
			tensor.rows[0][2] += -1.0f * point.x * point.z;
			tensor.rows[1][2] += -1.0f * point.y * point.z;

			tensor.rows[1][0] += -1.0f * point.x * point.y;
			tensor.rows[2][0] += -1.0f * point.x * point.z;
			tensor.rows[2][1] += -1.0f * point.y * point.z;


			sample_count++;
			}
		}
	}
	tensor *= 1.0f / (float)sample_count;
	return tensor;
}

//Signed Volumes - Prior to learning GJK algorithm , a predeccessor algorithm not utilised anymore is signed volume algorithm.
//In order for GJK to work , it needs the projection of origins to the simplexes we use, The reason for this since GJK needs to know
//the shortest distance/direction to an origin for any given simplex

//Simplex - In geometry a simplex is a generalized term for the notion of a triangle , tetrahedron to arbitrary dimensions , it is named so 
//because it simplex represents the simplest possible polytope made with line segments , meaning the line segment based shapes 
// that are the simplest shapes that can be created are considered simplexes (polytope is polygonal shapes , with flate sides/faces that
//are also 3D)
//The reason we need simplexes anyway is to know the shortest distance and direction to the origin from all simplexes. Hence why we are creating
//signed volume functions , that will allow us to find out these values in accordance to the GJK algorithm.
// 1 - Line Segment 
// 2 - Triangle 
// 3 - Tetragedron
//The signed volume approach is to find which axis-alligned plane maxmizes the projected area or length of a simplex. E.G
//Lets say we have a 1-simplex / line segment , we project this onto the x, y and z axis , we then determine
//which projection on each axis has the greatest length. We choose that axis , then project our point onto it as well
// this given we can calcyulate the barycentric coordinates of the point onto 1-simplex. giving us the project of the point onto the simplex.

Vec2 SignedVolume1D(const Vec3& s1, const Vec3& s2)
{
	//Calculated difference between both points , ray from a - b
	Vec3 ab = s2 - s1;

	//Calculated , ray from a  to origin
	Vec3 ap = Vec3(0.0f) - s1;

	//Projection of origin onto the line
	Vec3 p0 = s1 + ab * ab.Dot(ap) / ab.GetLengthSqr();


	//Choosing the axis with greatest length/distance

	int id = 0;
	float mu_max = 0;
	for (int i = 0; i < 3; i++)
	{
		//Current ray distance , used in condition
		float mu = s2[i] - s1[i];
		if(mu * mu > mu_max * mu_max)
		{
			//Give us line segment / simple with greatest length/distance
			mu_max = mu;
			id = i;

		}
	}

	//Project the simplex points and projected origin onto the "axis" with the greastest length

	const float a = s1[id];
	const float b = s2[id];
	const float p = p0[id];

	//Get the signed distance from a to p and from p to b
	const float C1 = p - a;
	const float C2 = b - p;

	//Checking between if p is between a or b
	if ((p > a && p < b) || (p > b && p < a))
	{
		//lambdas - anomynous function
		Vec2 lambdas;
		lambdas[0] = C2 / mu_max;
		lambdas[1] = C1 / mu_max;
		return lambdas;
	}

	//if p is on the far side of a, we return the following
	if ((a  <= b && p <= a) || (a >= b && p >= a))
	{
		return Vec2(1.0f, 0.0f);
	}

	//p must be on far side of b
	return Vec2(0.0f, 1.0f);
}

//For 2-simplex / triangle , we do the same thing , however only we project the triangle onto the xy . yz and zx plane. we then 
//determine which one has the greatest area. then projecting that point onto that plane
//if the point is inside the traingle, we calculate the barycentric coordinates and be done , if not we then project the point onto the edges of
//of the triangle using 1-simplex triangle projection method , and ise the closest projection

int CompareSigns(float a, float b)
{
	if (a > 0.0f && b > 0.0f)
	{
		return 1;
	}
	if (a < 0.0f && b < 0.0f)
	{
		return 1;
	}
	return 0;
}

Vec3 SignedVolume2D(const Vec3& s1, const Vec3& s2, const Vec3& s3)
{
	Vec3 normal = (s2 - s1).Cross(s3 - s1);

	Vec3 p0 = normal * s1.Dot(normal) / normal.GetLengthSqr();

	//Find the axis with the greatest projected area
	int id = 0;
	float max_area = 0;

	for (int i = 0; i < 3; i++)
	{
		int j = (i + 1) % 3;
		int k = (i + 2) % 3;

		Vec2 a = Vec2(s1[j], s1[k]);
		Vec2 b = Vec2(s2[j], s2[k]);
		Vec2 c = Vec2(s3[j], s3[k]);
	
		Vec2 ab = b - a;
		Vec2 ac = c - a;

		float area = ab.x * ac.y - ab.y * ac.x;
		if (area * area > max_area * max_area)
		{
			id = 0;
			max_area = area;
		}
	}

	//Project onto the appropriate axis
	int x = (id + 1) % 3;
	int y = (id + 2) % 3;

	Vec2 s[3];
	s[0] = Vec2(s1[x], s1[y]);
	s[1] = Vec2(s2[x], s2[y]);
	s[2] = Vec2(s3[x], s3[y]);

	Vec2 p = Vec2(p0[x], p0[y]);

	//Get the sub-areas of the triangles formed from the projected origin and edges
	Vec3 areas;
	for (int i = 0; i < 3; i++)
	{
		int j = (i + 1) % 3;
		int k = (i + 2) % 3;

		Vec2 a = p;
		Vec2 b = s[j];
		Vec2 c = s[k];

		Vec2 ab = b - a;
		Vec2 ac = c - a;

		//Calculating all the areas
		areas[i] = ab.x * ac.y - ab.y * ac.x;
	}

	//If the projected origin is inside the triangle , then returns the barycentric points
	if (CompareSigns(max_area , areas[0]) >  0 && CompareSigns(max_area , areas[1]) > 0 && CompareSigns(max_area , areas[2]) > 0)
	{
		Vec3 lambdas = areas / max_area;
		return lambdas;
	
	
	}

	//If previous condition is false , we need to project onto the egdes and determine
	//the closest point to the edges

	float distance = 1e10;
	Vec3 lambdas = Vec3(1.0f, 0.0f, 0.0f);

	for (int i = 0; i < 3; i++)
	{
		int k = (i + 1) % 3;
		int I = (i + 2) % 3;

		Vec3 edges_points[3];
		edges_points[0] = s1;
		edges_points[1] = s2;
		edges_points[2] = s3;

		Vec2 lambda_edge = SignedVolume1D(edges_points[k], edges_points[I]);
		Vec3 point = edges_points[k] * lambda_edge[0] + edges_points[I] * lambda_edge[1];

		if (point.GetLengthSqr() < distance)
		{
			distance = point.GetLengthSqr();
			lambdas[i] = 0;
			lambdas[k] = lambda_edge[0];
			lambdas[I] = lambda_edge[1];
		}
	}
	return lambdas;

}

//Final 3-simplex / tetrahedron function to calculate its barycentric coordinates ,if it isnt
//check each face for which projection is closest

Vec4 SignedVolume3D(const Vec3& s1, const Vec3& s2, const Vec3& s3, const Vec3& s4)
{
	Mat4 matrix;
	matrix.rows[0] = Vec4(s1.x, s2.x, s3.x, s4.x);
	matrix.rows[1] = Vec4(s1.y, s2.y, s3.y, s4.y);
	matrix.rows[2] = Vec4(s1.z, s2.z, s3.z, s4.z);
	matrix.rows[3] = Vec4(1.0f, 1.0f, 1.0f, 1.0f);

	Vec4 C4;
	C4[0] = matrix.Cofactor(3, 0);
	C4[0] = matrix.Cofactor(3, 1);
	C4[0] = matrix.Cofactor(3, 2);
	C4[0] = matrix.Cofactor(3, 3);

	const float determinant_matrix = C4[0] + C4[1] + C4[2] + C4[3];

	//If the barycentric coordinates put the origin inside the simplex, then return them
	if (CompareSigns(determinant_matrix , C4[0]) > 0 && CompareSigns(determinant_matrix , C4[1]) > 0 
		&& CompareSigns(determinant_matrix , C4[2]) > 0 && CompareSigns(determinant_matrix, C4[3]) > 0)
	{
		Vec4 lambdas = C4 * (1.0f / determinant_matrix);
		return lambdas;
	}

	//IF we get here , we need to project the origin onto the faces and determine
	// the closest one
	Vec4 lambdas;
	float distance = 1e10;
	for (int i = 0; i < 4; i++)
	{
		int k = (i + 1) % 4;
		int I = (i + 2) % 4;

		Vec3  face_points[4];
		face_points[0] = s1;
		face_points[1] = s2;
		face_points[2] = s3;
		face_points[3] = s4;
	
		Vec3 lambdas_face = SignedVolume2D(face_points[i], face_points[j], face_points[k]);
		Vec3 point = face_points[i] * lambdas_face[0] + face_points[j] * lambdas_face[1] + face_points[k]
			* lambdas_face[2];

		if (point.GetLengthSqr() < distance)
		{
			distance = point.GetLengthSqr();
			lambdas[i] = lambdas_face[0];
			lambdas[j] = lambdas_face[1];
			lambdas[k] = lambdas_face[2];
		}
	}

	return lambdas;
}

//Test Utility functions will be used to make sure the functions work as expected, projecting a point onto a simplex and return
//barycentric coordinates of this stated projection.
void TestSignedVolumeProjection()
{
	const Vec3 original_points[4] =
	{
		Vec3(0.0f , 0.0f , 0.0f) ,
		Vec3(1.0f , 0.0f , 0.0f) ,
		Vec3(0.0f , 1.0f , 0.0f) ,
		Vec3(0.0f , 0.0f , 1.0f) ,
	};
	Vec3 points[4];
	Vec4 lambdas;
	Vec3 v;

	for (int i = 0; i < 4; i++)
	{
		points[i] = original_points[i] + Vec3(1.0f, 1.0f, 1.0f);
	}

	lambdas = SignedVolume3D(points[0], points[1] , points[2] , points[3]);
	v.Zero();

	for (int i = 0; i < 4; i++)
	{
		v += points[i] * lambdas[i];
	}

	//print function here


	for (int i = 0; i < 4; i++)
	{
		points[i] = original_points[i] + Vec3(-1.0f, -1.0f, -1.0f) * 0.25f;

	}
	lambdas = SignedVolume3D(points[0], points[1], points[2], points[3]);
	v.Zero();
	for (int i = 0; i < 4; i++)
	{
		v += points[i] * lambdas[i];
	}

	for (int i = 0; i < 4; i++)
	{
		points[i] = original_points[i] + Vec3(-1.0f, -1.0f, -1.0f) ;

	}
}