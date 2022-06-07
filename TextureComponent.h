#pragma once
//ShaderComponent Depenencies
#include "Render.h"
#include "DDSTextureLoader.h"
#include "OBJLoader.h"
#include "resource.h"

//Libraries used to create application  

#include <windows.h>     //Window Library - Access to window functions
#include <d3d11_1.h>     //Core DX11 Library - Access DX11 Functions
#include <d3dcompiler.h> //Compile Library - Compiler for DX11 Code

#include <directxmath.h> //DX11 Math Library 
#include <directxcolors.h>//Color Math Library


using namespace DirectX; //Use default DX11 Naming conventions

class TextureComponent
{
public:
	//Constructor
	TextureComponent();
	//Destructor
	~TextureComponent();

	//Creates Texture
	HRESULT CreateTexture(wchar_t* filepath, ID3D11ShaderResourceView** texture, RenderCommands* render_command);

	//Binds an array(vector in our case) of resources to be processed to PS . Binding textures from a texture vector onto PS
	void BindTextures(int startSlot, int count, std::vector<ID3D11ShaderResourceView*> textures, RenderCommands* render_command);


};

