#pragma once

#include <d3d11_1.h>
#include <directxmath.h>
#include "ShaderCompiler.h"
using namespace DirectX;
struct VertexShader
{
	ID3D11InputLayout* _VertexLayout;
	ID3D11VertexShader* _VertexShader;
	WCHAR* _File;

	VertexShader() = default;

	VertexShader(WCHAR* file, ID3D11Device* device, ID3D11DeviceContext* device_context)
	{
		HRESULT hr;

		ID3D11InputLayout* vertex_layout;

		ID3DBlob* pVSBlob = nullptr;

		ShaderCompiler* _ShaderCompiler = new ShaderCompiler(file, "VS", "vs_4_0", &pVSBlob);

		// Create the pixel shader
		hr = device->CreateVertexShader(pVSBlob->GetBufferPointer(), pVSBlob->GetBufferSize(), nullptr, &_VertexShader);

		if (FAILED(hr))
		{
			MessageBox(nullptr, L"The FX File Vertex Shader Cannot be Compiled.  Please run this executable from the directory that contains the FX file.", L"Error", MB_OK);

		}


		// Define the input layout for data entering the Vertex Buffer
		D3D11_INPUT_ELEMENT_DESC layout[] =
		{
			{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
			{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
			{ "TEXCOORD" , 0 , DXGI_FORMAT_R32G32_FLOAT , 0 , 24 , D3D11_INPUT_PER_VERTEX_DATA , 0},
		};

		UINT numElements = ARRAYSIZE(layout);

		// Create the input layout - Describes layout of input buffer data within input assembler stage.
		device->CreateInputLayout(layout, numElements, pVSBlob->GetBufferPointer(), pVSBlob->GetBufferSize(), &vertex_layout);
		pVSBlob->Release();


		device_context->IASetInputLayout(vertex_layout);


	}

	ID3D11VertexShader* GetVertexShader()
	{
		return _VertexShader;
	}

};
