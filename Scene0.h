#pragma once
#include "SceneManager.h"
#include "Shape.h"
#include <string>


//Struct Used For Interpentration , utilising points of contacts between two bodies 
struct Contact
{
	Vec3 ptOnA_WorldSpace;
	Vec3 ptOnB_WorldSpace;

	Vec3 ptOnA_LocalSpace;
	Vec3 ptOnB_LocalSpace;
	

	//In World Space Coordinates , Normalizzed Direction From Point A-B Either World Or Local Space
	Vec3 Normal;

	//Value positive when non-pentrating , negative when penetratinhg
	float _SeperationDistance;
	float _TimeOfImpact;

	 Body* _BodyA;
	 Body* _BodyB;
};


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

		_Camera_Position = XMFLOAT3(0.0f, 20.0f, -50.0f);
		_Camera_Direction = XMFLOAT3(0.0f, -0.05f, 0.05f);
		_SceneCamera = new Camera(_Camera_Position, _Camera_Direction);

		Body body;
		body._Orientation = Quat(0.0f, 0.0f, 0.0f, 1.0f);
		body._Position = Vec3(0.0f, 0.0f, 0.0f);
		body._InvMass = 1.0f;
		body._Elasicity = 0.5f;
		body._Shape = new ShapeSphere(1.0f, surface, _pRenderCommand, _Tex,
		XMFLOAT3(0.0f, 1.0f, 0.0f), XMFLOAT3(1.0f, 1.0f, 1.0f), XMFLOAT3(0.0f, 0.0f, 0.0f));
		
		_SceneBodies.push_back(body);

		body._Orientation = Quat(0.0f, 0.0f, 0.0f, 1.0f);
		body._InvMass = 0.0f;
		body._Elasicity = 1.0f;

		body._Position = Vec3(0.0f, -50.0f, 0.0f);
		body._Shape = new ShapeSphere(1.0f, surface, _pRenderCommand, _Tex,
			XMFLOAT3(0.0f, 1.0f, 0.0f), XMFLOAT3(1.0f, 1.0f, 1.0f), XMFLOAT3(0.0f, 0.0f, 0.0f));
		
		_SceneBodies.push_back(body);


	}
	void Update(float delta_time) override
	{
		DrawUI();
		_SceneCamera->UpdateCamera();
		//Update Then Draw
		for (int i = 0; i < _SceneBodies.size(); i++)
		{
			Body* body = &_SceneBodies[i];

			//Gravity Needs To Be An Impulse 
			// i = DP , F = dp/dt => dp = F * DT => I = F * dt
			// F = MGS
			float mass = 1.0f / body->_InvMass;

			Vec3 impulse_gravity = Vec3(0.0f, -0.005f, 0.0f) * mass * 0.01f;
			body->AddImpulseLinear(impulse_gravity);
		}

		for (int i = 0; i < _SceneBodies.size(); i++)
		{
			for (int j = i + 1; j < _SceneBodies.size(); j++)
			{
			
				Body* bodyA = &_SceneBodies[i];
				Body* bodyB = &_SceneBodies[j];

				//Skip If Inf Mass
				if (0.0f == bodyA->_InvMass && 0.0f == bodyB->_InvMass)
				{
					continue;
				}
				Contact contact;
				if (Intersect(bodyA, bodyB , contact))
				{
					ResolveContacts(contact);
				}
			
			}
		}
		for (int l = 0; l < _SceneBodies.size(); l++)
		{
			Body* body = &_SceneBodies[l];
			//Keep This Hear - Object Wont move if Contact Is True, Since Velocity is nullified
			body->_Position += body->_LinearVelocity * 0.01f;
			body->_Shape->_Object->_ObjectTransformation.SetTranslation(XMFLOAT3(body->_Position.x, body->_Position.y, body->_Position.z));
			body->_Shape->_Object->_ObjectTransformation.UpdateObject();
			body->_Shape->_Object->Draw(_SceneCamera);

		}
	}

	void ResolveContacts(Contact contact)
	{
		Body* A = contact._BodyA;
		Body* B = contact._BodyB;

		const float invMassA = A->_InvMass;
		const float invMassB = B->_InvMass;

		const float elasicityA = A->_Elasicity;
		const float elasicityB = B->_Elasicity;
		const float TotalElasicity = elasicityA * elasicityB;
		//Calculate Collision Impulse
		const Vec3& n = contact.Normal;
		const Vec3 vab = A->_LinearVelocity - B->_LinearVelocity;

		const float ImpulseJ = -(1.0f + TotalElasicity) * vab.Dot(n) / (invMassA + invMassB);
		const Vec3 VecImpulseJ = n * ImpulseJ;

		A->AddImpulseLinear(VecImpulseJ * 1.0f);
		B->AddImpulseLinear(VecImpulseJ * -1.0f);

		const float tA = A->_InvMass / (A->_InvMass + B->_InvMass);
		const float tB = B->_InvMass / (A->_InvMass + B->_InvMass);

		const Vec3 ds = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;

		A->_Position += ds * tA;
		B->_Position -= ds * tB;


		A->_Shape->_Object->_ObjectTransformation.SetTranslation(XMFLOAT3(A->_Position.x, A->_Position.y, A->_Position.z));
		B->_Shape->_Object->_ObjectTransformation.SetTranslation(XMFLOAT3(B->_Position.x, B->_Position.y, B->_Position.z));

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
		if (GetAsyncKeyState('S')) {
			Body* body = &_SceneBodies[0];
			body->_Position.y = 10.0f;

		}
	}

	bool Intersect( Body* A,  Body* B , Contact & contact)
	{

		contact._BodyA = A;
		contact._BodyB = B;

		Vec3 ab = B->_Position - A->_Position;

		//Assign Normal Distance Between Objects
		contact.Normal = ab;
		contact.Normal.Normalize();


		const ShapeSphere* a_sphere = (const ShapeSphere*)A->_Shape;
		const ShapeSphere* b_sphere = (const ShapeSphere*)B->_Shape;


		contact.ptOnA_WorldSpace = A->_Position + contact.Normal * a_sphere->_Radius;
		contact.ptOnB_WorldSpace = B->_Position - contact.Normal * a_sphere->_Radius;


		const float radiusAB = a_sphere->_Radius + b_sphere->_Radius;
		const float lengthSquare = ab.GetLengthSqr();

		if (lengthSquare <= (radiusAB * radiusAB))
		{
			return true;
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


