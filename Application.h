#pragma once
#include "ConstantStructure.h"
#include "Mesh.h"
#include "OBJLoader.h"

#include <dinput.h>

#pragma comment (lib, "dinput8.lib")
#pragma comment (lib, "dxguid.lib")

//Access to core DX functions 

#include "ImGuiManager.h"
#include "SceneManager.h"


#include <d3d11_1.h>

//Allows us to compile DX11 Code
#include <d3dcompiler.h>
//Math Library
#include <directxmath.h>
#include <directxcolors.h>
//Libraries used to create application 

#include "DirectX.h"
#include "Render.h"
#include "TextureComponent.h"
#include "Scene0.h"

#include <vector>

#include "DDSTextureLoader.h"
#include "PixelShader.h"
#include "VertexShader.h"

#include "StaticCamera.h"


//Allows us to easily call reference upon our DX naming conventions
using namespace DirectX;

//XMFLOAT3 - Describes 3DVector , Consisting of Three Points (x,y ,z)
//XMFLOAT4 - Describes 4DVector , Consisting of Four Points (x)
//XMFLOAT4X4  - Structure that creates 4*4 Floating Point Matrix - Can be used to store XMMATRIX DATA
//and relevant matrix data within DX11


class Application
{
private:

	DX* _pDX11;

	RenderCommands* _pRenderCommands;
	TextureComponent* _Tex;
	SceneManager* _pSceneManager;

	PixelShader* _pPixelShader;
	VertexShader* _pVertexShader;

	ID3D11PixelShader* _PS;
	ID3D11VertexShader* _VS;

	ImGuiManager* _pGUIManager;

	//Stores all of our levels we use
	//Using smart pointers here as they automatically deallocate themselves when needed

	Scene0* _pScene0;


	StaticDefaultCamera* _StaticDefaultCamera;

	DWORD dwTimeStart = 0;
	std::vector<Camera*> _CameraObjects;

public:

	HINSTANCE  _hInst;								//Used to specify instance which the class is registred					
	HWND                    _hWnd;								//Used to handle a window , part of Win32 API , crates window using window instance above.

	IDirectInputDevice8* DIKeyBoard;
	IDirectInputDevice8* DIMouse;

	DIMOUSESTATE MouseLastState;
	LPDIRECTINPUT8 DirectInput;

	UINT _WindowHeight;											//Define window height
	UINT _WindowWidth;											//Define window width

	Application();
	~Application();


	HRESULT Initialise(HINSTANCE hInstance, int nCmdShow);

	SceneManager* _pCurrentScene;

	void Input();
	void Cleanup();
	void RenderFrame();
	void SwitchScene(SceneManager* scene);
	HRESULT InitialiseWindow(HINSTANCE hInstance, int nCmdShow);

};

