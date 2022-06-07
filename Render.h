#pragma once

//Used For Initialising DirectX FrameWork Components

#include <windows.h>     //Window Library - Access to window functions
#include <d3d11_1.h>     //Core DX11 Library - Access DX11 Functions
#include <d3dcompiler.h> //Compile Library - Compiler for DX11 Code

#include <directxmath.h> //DX11 Math Library 
#include <directxcolors.h>//Color Math Library
#include "ConstantStructure.h"
#include "Lighting.h"

#include "StaticCamera.h"
#include "Surface.h"

using namespace DirectX; //Use default DX11 Naming conventions

class RenderCommands
{
public:

	RenderCommands(ID3D11Device* device, ID3D11DeviceContext* device_context, ID3D11Buffer* CB) : _pDevice(device), _pDeviceContext(device_context), _pConstantBuffer(CB)
	{
	}

	~RenderCommands()
	{
		Cleanup();
	}

	void ChangeBlendState2(ID3D11BlendState* blendstate)
	{
		float blendFactor[] = { 0.0f, 1.0f, 0.0f, 1.0f };
		_pDeviceContext->OMSetBlendState(blendstate, blendFactor, 0xffffffff);
	}

	void ChangeBlendState1(ID3D11BlendState* blendstate)
	{
		float blendFactor[] = { 0.75f, 0.75f, 0.75f, 1.0f };
		_pDeviceContext->OMSetBlendState(blendstate, blendFactor, 0xffffffff);
	}
	void ChangeBlendState3(ID3D11BlendState* blendstate)
	{
		float blendFactor[] = { 0.0f, 0.0f, 0.0f, 1.0f };
		_pDeviceContext->OMSetBlendState(blendstate, blendFactor, 0xffffffff);
	}
	void SetRasterState(ID3D11RasterizerState* rasterstate)
	{
		_pDeviceContext->RSSetState(rasterstate);
	}
	void  ClearRenderTarget(ID3D11RenderTargetView* target_view, ID3D11DepthStencilView* depth_view)
	{
		_pDeviceContext->ClearRenderTargetView(target_view, ClearColor);
		_pDeviceContext->ClearDepthStencilView(depth_view, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);
	}
	void SwapChainPresent(IDXGISwapChain* swap_chain)
	{

		swap_chain->Present(0, 0);
	}
	void BindVertexShader(ID3D11VertexShader* vertex_shader)
	{
		_pDeviceContext->VSSetShader(vertex_shader, nullptr, 0);

	}
	void BindPixelShader(ID3D11PixelShader* pixel_shader)
	{
		_pDeviceContext->PSSetShader(pixel_shader, nullptr, 0);

	}
	void BindSampler(ID3D11SamplerState* sampler)
	{
		_pDeviceContext->PSSetSamplers(0, 1, &sampler);
	}
	void UpdateConstantBuffer(Camera* camera , XMFLOAT4X4 world, Surface object_surface)
	{
		ConstantBuffer constantbuffer;
		Lighting basicLight;
		// Setup the scene's light





		XMMATRIX _world = XMLoadFloat4x4(&world);
		XMMATRIX view = XMLoadFloat4x4(&camera->GetView());
		XMMATRIX projection = XMLoadFloat4x4(&camera->GetProjection());

		constantbuffer.World = XMMatrixTranspose(_world);
		constantbuffer.View = XMMatrixTranspose(view);
		constantbuffer.Projection = XMMatrixTranspose(projection);

		constantbuffer.Mat.AmbientMtrl = object_surface.Ambient;
		constantbuffer.Mat.DiffuseMtrl = object_surface.Diffuse;
		constantbuffer.Mat.SpecularMtrl = object_surface.Specular;

		//Actual Light In Environment
		constantbuffer.Light.AmbientLight = basicLight.AmbientLight = XMFLOAT4(0.5f, 0.5f, 0.5f, 1.0f);
		constantbuffer.Light.DiffuseLight = basicLight.DiffuseLight = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
		constantbuffer.Light.SpecularLight = basicLight.SpecularLight = XMFLOAT4(0.8f, 0.8f, 0.8f, 1.0f);
		constantbuffer.Light.LightVecW = basicLight.LightVecW = XMFLOAT3(2.0f, 10.0f, -1.0f);
		constantbuffer.Light.SpecularPower = basicLight.SpecularPower = 20.0f;
		
		_pDeviceContext->UpdateSubresource(_pConstantBuffer, 0, nullptr, &constantbuffer, 0, 0);
	}


	ID3D11Device* GetDevice()
	{
		return _pDevice;

	}
	ID3D11DeviceContext* GetDeviceContext()
	{
		return _pDeviceContext;

	}
	void Cleanup()
	{
		delete(_pConstantBuffer);
		delete(_pDevice);
		delete(_pDeviceContext);
	}

private:

	ID3D11Buffer* _pConstantBuffer;								//Defines ConstantBuffer Storage 
	ID3D11Device* _pDevice;
	ID3D11DeviceContext* _pDeviceContext;
	//Holds background colour value
	float ClearColor[4] = { 0.0f,0.0f,1.0f,0.0f };
};

