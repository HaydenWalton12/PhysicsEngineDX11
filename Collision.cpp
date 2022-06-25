#include "Collision.h"
#include "ShapeSphere.h"

Collision::Collision()
{
}

Collision::~Collision()
{
}



bool Collision::RaySphere(Vec3 ray_start, Vec3& ray_direction, Vec3 sphere_centre, float sphere_radius, float& t1, float& t2)
{
	Vec3 m = sphere_centre - ray_start;
	float a = ray_direction.Dot(ray_direction);
	float b = m.Dot(ray_direction);
	float c = m.Dot(m) - sphere_radius * sphere_radius;

	float delta = b * b - a * c;

	float invA = 1.0f / a;

	if (delta < 0)
	{
		return false;
	}

	float delta_root = sqrtf(delta);

	t1 = invA * (b - delta_root);
	t2 = invA * (b + delta_root);

	return true;

}
bool Collision::SphereSphereDynamic(ShapeSphere* sphere_a, ShapeSphere* sphere_b, Vec3& pos_a, Vec3& pos_b, Vec3& vel_a, Vec3& vel_b, float dt, Vec3& pt_On_A, Vec3& pt_On_B, float& toi)
{

	Vec3 relative_velocity = vel_a - vel_b;

	Vec3 start_PT_a = pos_a;
	Vec3 end_PT_a = pos_a + relative_velocity * dt;

	Vec3 ray_direction = end_PT_a - start_PT_a;

	float t0 = 0;
	float t1 = 0;

	if (ray_direction.GetLengthSqr() < 0.001f * 0.001f)
	{
		Vec3 ab = pos_b - pos_a;

		float radius = sphere_a->_Radius + sphere_b->_Radius + 0.001f;
		if (ab.GetLengthSqr() < radius * radius)
		{
			return false;
		}

	}
	else if (!RaySphere(pos_a, ray_direction, pos_b, sphere_a->_Radius + sphere_b->_Radius, t0, t1))
	{
		return false;
	}

	t0 *= dt;
	t1 *= dt;

	if (t1 < 0.0f)
	{
		return false;

	}
	toi = (t0 < 0.0f) ? 0.0f : t0;

	if (toi > dt)
	{
		return false;
	}

	Vec3 newPosA = pos_a + vel_a * toi;
	Vec3 newPosB = pos_b + vel_b * toi;
	Vec3 ab = newPosB - newPosA;
	ab.Normalize();

	pt_On_A = newPosA + ab * sphere_a->_Radius;
	pt_On_B = newPosB - ab * sphere_b->_Radius;

	return true;
}

bool Collision::Intersect(Body* A, Body* B, float dt, Contact& contact)
{
	contact._BodyA = A;
	contact._BodyB = B;

	if (A->_Shape->GetType() == Shape::SHAPE_SPHERE && B->_Shape->GetType() == Shape::SHAPE_SPHERE)
	{

		ShapeSphere* a_sphere = (ShapeSphere*)A->_Shape;
		ShapeSphere* b_sphere = (ShapeSphere*)B->_Shape;

		Vec3 pos_a = A->_Position;
		Vec3 pos_b = B->_Position;

		Vec3 vel_a = A->_LinearVelocity;
		Vec3 vel_b = B->_LinearVelocity;

		if (SphereSphereDynamic(a_sphere, b_sphere, pos_a, pos_b, vel_a, vel_b, dt, contact.ptOnA_WorldSpace, contact.ptOnB_WorldSpace, contact._TimeOfImpact))
		{

			A->Update(contact._TimeOfImpact);
			B->Update(contact._TimeOfImpact);

			contact.ptOnA_LocalSpace = A->WorldSpaceToBodySpace(contact.ptOnA_WorldSpace);
			contact.ptOnB_LocalSpace = B->WorldSpaceToBodySpace(contact.ptOnB_WorldSpace);

			contact.Normal = A->_Position - B->_Position;
			contact.Normal.Normalize();

			//Unqind Time Step
			A->Update(-contact._TimeOfImpact);
			B->Update(-contact._TimeOfImpact);


			Vec3 ab = B->_Position - A->_Position;
			float r = ab.GetMagnitude() - (a_sphere->_Radius + b_sphere->_Radius);

			contact._SeperationDistance = r;

			return true;
		}

	}
	return false;
}
