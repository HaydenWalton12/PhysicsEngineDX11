#pragma once
#include "SceneManager.h"
#include "Shape.h"
#include <string>


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


		_Camera_Position = XMFLOAT3(0.0f, 10.0f, -15.0f);
		_Camera_Direction = XMFLOAT3(0.0f, -0.5f, 0.5f);
		_SceneCamera = new Camera(_Camera_Position, _Camera_Direction);

		Body body;

		body._Orientation = Quat(0.0f, 0.0f, 0.0f, 1.0f);
		body._Position = Vec3(0.0f, 0.0f, 0.0f);
		body._Shape = new ShapeSphere(1.0f, surface, _pRenderCommand, _Tex,
		XMFLOAT3(0.0f, 1.0f, 0.0f), XMFLOAT3(1.0f, 1.0f, 1.0f), XMFLOAT3(0.0f, 0.0f, 0.0f));
		_SceneBodies.push_back(body);

	}
	void Update(float delta_time) override
	{
		DrawUI();
		_SceneCamera->UpdateCamera();

		for (int i = 0; i < _SceneBodies.size(); i++)
		{
			Body* body = &_SceneBodies[i];
			body->_Shape->_Object->_ObjectTransformation.UpdateObject();
			body->_Shape->_Object->Draw(_SceneCamera);

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

	std::vector<Body> _SceneBodies;
};


