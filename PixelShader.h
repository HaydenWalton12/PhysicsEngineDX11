#pragma once
#include <d3d11_1.h>
#include <directxmath.h>

using namespace DirectX;

#include "ShaderCompiler.h"


struct PixelShader
{
	ID3D11PixelShader* _PixelShader;

	PixelShader() = default;

	PixelShader(WCHAR* file, ID3D11Device* device)
	{
		HRESULT hr;

		ID3DBlob* pPSBlob = nullptr;
		ShaderCompiler* _ShaderCompiler = new ShaderCompiler(file, "PS", "ps_4_0", &pPSBlob);

		// Create the pixel shader
		hr = device->CreatePixelShader(pPSBlob->GetBufferPointer(), pPSBlob->GetBufferSize(), nullptr, &_PixelShader);

		if (FAILED(hr))
		{
			MessageBox(nullptr,
				L"The FX File Pixel Shader Cannot be Compiled.  Please run this executable from the directory that contains the FX file.", L"Error", MB_OK);
		}


	}
	ID3D11PixelShader* GetPixelShader()
	{
		return _PixelShader;
	}
};
