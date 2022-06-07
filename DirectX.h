#pragma once
/// <Summary>
/// 
/// <Summary>

#include "ConstantStructure.h"

#include <windows.h>     //Window Library - Access to window functions
#include <d3d11_1.h>     //Core DX11 Library - Access DX11 Functions
#include <d3dcompiler.h> //Compile Library - Compiler for DX11 Code

#include <directxmath.h> //DX11 Math Library 
#include <directxcolors.h>//Color Math Library


using namespace DirectX; //Use default DX11 Naming conventions

class DX
{
public:

	DX(UINT width, UINT height, HWND hWnd);
	~DX();
	void InitialiseDevice();									//Calls Graphical Components for application creation - Has to be in order disclosed , Function names are self-explanatory
	ID3D11RenderTargetView* _pRenderTargetView;				//Defines render target and access to sub-resources that can be accessed during rendering to a buffer ()
	ID3D11DepthStencilView* _pDepthStencilView;				//Defines view resource that accesses a texture resource during depth stencil testing , Our depth data is bound to a texture object
	ID3D11SamplerState* _pSamplerLinear;						//Bind to any shader stage (VS / PS) , used to bind reference of texture sample operations - We bind to PixelShader to accomidate per-pixel lighting

	ID3D11Device* _pDevice;									    //Virtual Representation of Video Card , used to create resources for application , resources exuted using device context
	ID3D11DeviceContext* _pDeviceContext;					    //Stores reference to DeviceContext , allow us to generate rendering commands to execute within application
	ID3D11Buffer* _pConstantBuffer;								//Defines ConstantBuffer Storage 
	IDXGISwapChain* _pSwapChain;						        //Interface object used to implement one or more buffers for storing rendered data before presenting it as an output
	ID3D11RasterizerState* _SolidRasterState;					//Holds interface description for rasterizer state - to be bound to rasterizer stage
	ID3D11RasterizerState* _WireFrameRasterState;
	ID3D11BlendState* _BlendState;
	D3D_DRIVER_TYPE			  _driverType;
private:


	D3D_FEATURE_LEVEL		  _featureLevel;					//Defines feature level targeted by the device pointer. Essentially defines the version of the DirectX API we want to use
	ID3D11ShaderResourceView* _pTextureRV;						//Defines shader subresource that can be accessed during rendering , e.g constant buffer,  in our case a texture buffer bound to texture data.
	ID3D11Texture2D* _pDepthStencilBuffer;				//2D texture interface manager , managing texel data (structured image data) , stores depth data , processed in DepthStencilView

private:

	HRESULT InitialiseSwapchain();								//Creates SwapChain Function - Refer to notes of what this is
	HRESULT InitialiseRenderTarget();							//Creates RenderTarget Function - Refer to notes of what this is

	void Cleanup();												//Cleans Member Object Values
	void InitialiseSampler();									//Creates Sampler  Function - Refer to notes of what this is
	void InitialiseDepth();										//Creates DepthBuffer Function - Refer to notes of what this is
	void InitialiseViewport();									//Initialise View Region - Refer to notes of what this is
	void InitialiseConstantBuffer();							//Initialise Constant Buffer - Refer to notes of what this is
	void InitialiseSolid();
	void InitialiseWireFrame();
	void InitialiseAlphaBlending();



	UINT _WindowHeight;											//Define window height
	UINT _WindowWidth;											//Define window width
	HWND _hWnd;
};

