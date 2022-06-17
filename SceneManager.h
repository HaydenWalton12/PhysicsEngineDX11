#pragma once
#include "Render.h"
#include "ImGuiManager.h"
#include "Object.h"
#include "TextureComponent.h"
#include <vector>

/// <summary>
/// Used to Hold/Manage the Functions of Scenes
/// Scenes Still hold individual functionality , however share the common functions
/// Allows to easily switch between scenes since they are similar in composition and are children to this scene manager.
/// </summary>

class SceneManager
{
public:
	SceneManager(RenderCommands* render_command, ImGuiManager* gui_manager, ID3D11Device* device) : _pRenderCommand(render_command), _pGUIManager(gui_manager), _Device(device)
	{

	}


	~SceneManager()
	{
		SceneCameras.clear();

		_pRenderCommand = nullptr;
		_pGUIManager = nullptr;
	}



	//Clears Scene Values
	virtual void ExitScene()
	{
		SceneCameras.clear();
	}


	virtual void Update(float delta_time)
	{

	}

	//Loads Scene Data Individually
	virtual void LoadScene()
	{



	}

	//Any Scene With Custom/Unique Input Functions
	virtual void PollInput(float delta_time)
	{

	}


	//Within Child Classes, Values are reset to default 
	//E.G Transform Properties Ect..
	virtual void ResetScene()
	{}
	ID3D11Device* _Device;
	std::vector<Camera*> SceneCameras;
private:

	RenderCommands* _pRenderCommand;							//Graphic Component Class pointer, needed to reference graphiccomponents for creation of object
	TextureComponent* _Tex;
	ImGuiManager* _pGUIManager;
};

