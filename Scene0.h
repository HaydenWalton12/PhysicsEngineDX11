#pragma once
#include "SceneManager.h"
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

	}

	//Instantiates All Scene Objects
	void LoadScene() override
	{


	}
	void Update(float delta_time) override
	{
		DrawUI();


	}




	void DrawUI()
	{
		ImGui::Begin("Physics Engine Simulations");
		//Menu Displayed After Simulation Starts
		ImGui::Text("About : This is a simple engine demonstrating the approximation of physics properties.\n\n\n\n");
		ImGui::Text("Scene 1:\nDemonstration of either constant velocity or acceleration , with further option\nof applying either laminar or turbulent drag.");
		ImGui::Text("");
		ImGui::Text("Scene 2:\nForce accumliation model , demonstrating thrust, drag , gravity and friction.");
		ImGui::Text("");
		ImGui::Text("Scene 3:\nAngular Velocity and Acceleration , implementing using quaternions via inertia and torque.");
		ImGui::Text("");
		ImGui::Text("Scene 4:\nNarrow Phase Collision Detection , using appropriate bounding volumes. With Further Applied Collision Response.");
		ImGui::Text("");
		ImGui::Text("Scene 5:\nParticle System Demonstration");
		ImGui::Text("");
		ImGui::Text("Main Controls:\nNumber 1 - Move To Scene1 \nNumber 2 - Move To Scene2\nNumber 3 - Move To Scene3\nNumber 4 - Move To Scene5\nNumber 5 - Move To Scene5");
		ImGui::Text("R - Reset Scene");
		ImGui::End();


	}


	void PollInput(float delta_time) override
	{

	}


private:

	HWND _HWND;
	RenderCommands* _pRenderCommand;
	TextureComponent* _Tex;
	ImGuiManager* _pGUIManager;


};


