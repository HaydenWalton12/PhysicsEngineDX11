#pragma once
#include "SceneManager.h"
#include "Shape.h"
#include "ShapeSphere.h"
#include <string>

#include "Collision.h"


class Scene0 : public SceneManager
{
public:
	Scene0(HWND hWnd, RenderCommands* render_command, TextureComponent* tex, ImGuiManager* gui_manager, ID3D11Device* device) : SceneManager(render_command, gui_manager, device)
	{
		_HWND = hWnd;
		_pRenderCommand = render_command;
		_Tex = tex;
		_pGUIManager = gui_manager;
		LoadScene();
	}

	//Instantiates All Scene Objects
	void LoadScene() override
	{

		XMFLOAT4 Ambient = XMFLOAT4(0.3f, 0.3f, 0.3f, 1.0f);
		XMFLOAT4 Diffuse = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
		XMFLOAT4 Specular = XMFLOAT4(0.5f, 0.5f, 0.5f, 1.0f);

		FLOAT SpecularPower = 10.0f;

		Surface surface = Surface(Ambient, Diffuse, Specular, SpecularPower);

		_Camera_Position = XMFLOAT3(0.0f, 10.0f, -10.0f);
		_Camera_Direction = XMFLOAT3(0.0f, -0.01f, 0.01f);
		_SceneCamera = new Camera(_Camera_Position, _Camera_Direction);


		Body body;
				body._Position = Vec3(0.0f, 10.0f, 0.0f);
				body._Orientation = Quat(0, 0, 0, 1);
				body._LinearVelocity.Zero();
				body._InvMass = 1.0f;
				body._Elasicity = 0.5f;
				body._Friction = 0.5f;
				body._Shape = new ShapeSphere(1.0f, surface, _pRenderCommand, _Tex,
					XMFLOAT3(0.0f, 10.0f, 0.0f), XMFLOAT3(1.0f, 1.0f, 1.0f), XMFLOAT3(0.0f, 0.0f, 0.0f));;
				_SceneBodies.push_back(body);

				body._Position = Vec3(0.0f, 15.0f, 0.0f);
				body._Orientation = Quat(0, 0, 0, 1);
				body._LinearVelocity.Zero();
				body._InvMass = 1.0f;
				body._Elasicity = 0.5f;
				body._Friction = 0.5f;
				body._Shape = new ShapeSphere(1.0f, surface, _pRenderCommand, _Tex,
					XMFLOAT3(0.0f, 15.0f, 0.0f), XMFLOAT3(1.0f, 1.0f, 1.0f), XMFLOAT3(0.0f, 0.0f, 0.0f));;
				_SceneBodies.push_back(body);
	}

	void Update(float delta_time) override;
	bool RaySphere(Vec3 ray_start , Vec3 & ray_direction , Vec3 sphere_centre , float sphere_radius , float & t1, float & t2)
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

	bool SphereSphereDynamic(ShapeSphere* sphere_a, ShapeSphere* sphere_b, Vec3& pos_a, Vec3& pos_b, Vec3& vel_a, Vec3& vel_b , float dt , Vec3& pt_On_A , Vec3& pt_On_B ,float& toi)
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
		else if (!RaySphere(pos_a , ray_direction , pos_b , sphere_a->_Radius + sphere_b->_Radius , t0 , t1) )
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

	void ResolveContacts(Contact contact)
	{
		Body* A = contact._BodyA;
		Body* B = contact._BodyB;

		Vec3 ptOnA = A->BodySpaceToWorldSpace(contact.ptOnA_LocalSpace);
		Vec3 ptOnB = B->BodySpaceToWorldSpace(contact.ptOnB_LocalSpace);


		const float elasicityA = A->_Elasicity;
		const float elasicityB = B->_Elasicity;
		const float TotalElasicity = elasicityA * elasicityB;

		const float invMassA = A->_InvMass;
		const float invMassB = B->_InvMass;

		Mat3 invWorldInertiaA = A->GetInverseInertiaTensorWorldSpace();
		Mat3 invWorldInertiaB = B->GetInverseInertiaTensorWorldSpace();

		//Calculate Collision Impulse
		const Vec3 n = contact.Normal;

		Vec3 ra = ptOnA - A->GetCenterOfMassWorldSpace();
		Vec3 rB = ptOnB - B->GetCenterOfMassWorldSpace();

		Vec3 angularJA = (invWorldInertiaA * ra.Cross(n) ).Cross(ra);
		Vec3 angularJB = (invWorldInertiaB * ra.Cross(n) ).Cross(rB);
		
		float angular_factor = (angularJA + angularJB).Dot(n);

		Vec3 velA = A->_LinearVelocity + A->_AngularVelocity.Cross(ra);
		Vec3 velB = B->_LinearVelocity + B->_AngularVelocity.Cross(rB);

		//Calcvulate Collision Impulse
		Vec3 vab = velA - velB;

		float impulseJ = (1.0f + TotalElasicity) * vab.Dot(n) / (invMassA + invMassB + angular_factor);
		const Vec3 VecImpulseJ = n * impulseJ;

		A->ApplyImpulse(ptOnA,VecImpulseJ * -1.0f);
		B->ApplyImpulse(ptOnB,VecImpulseJ * 1.0f);

		//Calculate The Impulse based upon friction

		float frictionA = A->_Friction;
		float frictionB = B->_Friction;

		float TotalFriction = frictionA * frictionB;

		Vec3 velNorm = n * n.Dot(vab);

		Vec3 velTang = vab - velNorm;

		Vec3 relativeVelTang = velTang;
		relativeVelTang.Normalize();

		Vec3 inertiaA = (invWorldInertiaA * ra.Cross(relativeVelTang)).Cross(ra);
		Vec3 inertiaB = (invWorldInertiaB * rB.Cross(relativeVelTang)).Cross(rB);
		float invInertia = (inertiaA + inertiaB).Dot(relativeVelTang);

		float reducedMass = 1.0f / (A->_InvMass + B->_InvMass + invInertia);
		Vec3 impulseFricion = velTang * reducedMass * TotalFriction;

		//Kinetic Friction

		A->ApplyImpulse(ptOnA, impulseFricion * -1.0f);
		B->ApplyImpulse(ptOnB, impulseFricion * 1.0f);


		if (0.0f == contact._TimeOfImpact)
		{
			const Vec3 ds = ptOnB - ptOnA;

			const float tA = A->_InvMass / (A->_InvMass + B->_InvMass);
			const float tB = B->_InvMass / (A->_InvMass + B->_InvMass);

			A->_Position += ds * tA;
			B->_Position -= ds * tB;

			A->_Shape->_Object->_ObjectTransformation.SetTranslation(XMFLOAT3(A->_Position.x, A->_Position.y, A->_Position.z));
			B->_Shape->_Object->_ObjectTransformation.SetTranslation(XMFLOAT3(B->_Position.x, B->_Position.y, B->_Position.z));
		}




	}
	void DrawUI()
	{
		ImGui::Begin("Physics Engine Simulations");
		//Menu Displayed After Simulation Starts
		ImGui::Text("About : This is a simple engine demonstrating the approximation of physics properties.\n\n\n\n");
		//ImGui::Text("Scene 1:\nDemonstration of either constant velocity or acceleration , with further option\nof applying either laminar or turbulent drag.");
		//ImGui::Text("");
		//ImGui::Text("Scene 2:\nForce accumliation model , demonstrating thrust, drag , gravity and friction.");
		//ImGui::Text("");
		//ImGui::Text("Scene 3:\nAngular Velocity and Acceleration , implementing using quaternions via inertia and torque.");
		//ImGui::Text("");
		//ImGui::Text("Scene 4:\nNarrow Phase Collision Detection , using appropriate bounding volumes. With Further Applied Collision Response.");
		//ImGui::Text("");
		//ImGui::Text("Scene 5:\nParticle System Demonstration");
		//ImGui::Text("");
		//ImGui::Text("Main Controls:\nNumber 1 - Move To Scene1 \nNumber 2 - Move To Scene2\nNumber 3 - Move To Scene3\nNumber 4 - Move To Scene5\nNumber 5 - Move To Scene5");
		//ImGui::Text("R - Reset Scene");
		ImGui::End();
	}
	void PollInput(float delta_time) override
	{
		if (GetAsyncKeyState('W')) {
			Body* body = &_SceneBodies[0];
			body->AddImpulseLinear(Vec3(0.0f, 1.0f, 0.0f));

		}
		if (GetAsyncKeyState('A')) {
			Body* body = &_SceneBodies[0];
			body->AddImpulseLinear(Vec3(-1.0f, 0.0f, 0.0f));

		}
		if (GetAsyncKeyState('E')) {
			Body* body = &_SceneBodies[0];
			body->AddImpulseLinear(Vec3(0.0f, 0.0f, 1.0f));

		}
		if (GetAsyncKeyState('R')) {
			Body* body = &_SceneBodies[0];
			body->AddImpulseLinear(Vec3(0.0f, 0.0f, -1.0f));

		}
		if (GetAsyncKeyState('D')) {
			Body* body = &_SceneBodies[0];
			body->AddImpulseLinear(Vec3(1.0f, 0.0f, 0.0f));

		}
		if (GetAsyncKeyState('S')) {
			Body* body = &_SceneBodies[0];
			body->AddImpulseLinear(Vec3(0.0f, -1.0f, 0.0f));

		}
		if (GetAsyncKeyState('X')) {
			Body* body = &_SceneBodies[0];
			body->AddImpulseAngular(Vec3(0.0f, 0.001f, 0.0f));

		}
		if (GetAsyncKeyState('Z')) {
			Body* body = &_SceneBodies[0];
			body->AddImpulseAngular(Vec3(0.0f, -0.001f, 0.0f));

		}
		if (GetAsyncKeyState('C')) {
			Body* body = &_SceneBodies[0];
			body->_Position = Vec3(0.0f, 0.001f, 0.0f);

		}
	}
	bool Intersect( Body* A,  Body* B, float dt , Contact & contact)
	{
		contact._BodyA = A;
		contact._BodyB = B;

		if (A->_Shape->GetType() == Shape::SHAPE_SPHERE && B->_Shape->GetType() == Shape::SHAPE_SPHERE)
		{

		 ShapeSphere* a_sphere = ( ShapeSphere*)A->_Shape;
		 ShapeSphere* b_sphere = ( ShapeSphere*)B->_Shape;

			Vec3 pos_a = A->_Position;
			Vec3 pos_b = B->_Position;

			Vec3 vel_a = A->_LinearVelocity;
			Vec3 vel_b = B->_LinearVelocity;

			if (SphereSphereDynamic(a_sphere, b_sphere, pos_a, pos_b, vel_a, vel_b, dt, contact.ptOnA_WorldSpace, contact.ptOnB_WorldSpace, contact._TimeOfImpact))
			{
			
				A->Update(contact._TimeOfImpact);
				B->Update(contact._TimeOfImpact );

				contact.ptOnA_LocalSpace = A->WorldSpaceToBodySpace(contact.ptOnA_WorldSpace);
				contact.ptOnB_LocalSpace = B->WorldSpaceToBodySpace(contact.ptOnB_WorldSpace);

				contact.Normal = A->_Position - B->_Position;
				contact.Normal.Normalize();

				//Unqind Time Step
				A->Update(-contact._TimeOfImpact );
				B->Update(-contact._TimeOfImpact);


				Vec3 ab = B->_Position - A->_Position;
				float r = ab.GetMagnitude() - (a_sphere->_Radius + b_sphere->_Radius);

				contact._SeperationDistance = r;

				return true;
			}

		}
		return false;
	}
	




private:
	Camera* _SceneCamera;

	XMFLOAT3 _Camera_Position;
	XMFLOAT3 _Camera_Direction;

	HWND _HWND;
	RenderCommands* _pRenderCommand;
	TextureComponent* _Tex;
	ImGuiManager* _pGUIManager;

	Body body;

	Object* _Sphere;

	Object* _Plane;

	std::vector<Body> _SceneBodies;
};


