#pragma once

#include <windows.h>     //Window Library - Access to window functions
#include <d3d11_1.h>     //Core DX11 Library - Access DX11 Functions
#include <d3dcompiler.h> //Compile Library - Compiler for DX11 Code

#include <directxmath.h> //DX11 Math Library 
#include <directxcolors.h>//Color Math Library


using namespace DirectX; //Use default DX11 Naming 


class ShaderCompiler
{
public:
    ShaderCompiler(WCHAR* file_path, LPCSTR szEntryPoint, LPCSTR szShaderModel, ID3DBlob** ppBlobOut)
    {
        HRESULT hr;

        // Compile the pixel shader
        hr = CompileShaderFromFile(file_path, szEntryPoint, szShaderModel, ppBlobOut);
        //Check Error Method - Was the CompiledShaderFromFile Above correct?

    }

    HRESULT CompileShaderFromFile(WCHAR* file_path, LPCSTR szEntryPoint, LPCSTR szShaderModel, ID3DBlob** ppBlobOut)
    {
        HRESULT hr = S_OK;

        DWORD dwShaderFlags = D3DCOMPILE_ENABLE_STRICTNESS;
#if defined(DEBUG) || defined(_DEBUG)
        // Set the D3DCOMPILE_DEBUG flag to embed debug information in the shaders.
        // Setting this flag improves the shader debugging experience, but still allows 
        // the shaders to be optimized and to run exactly the way they will run in 
        // the release configuration of this program.
        dwShaderFlags |= D3DCOMPILE_DEBUG;
#endif
        //Used to return arbitrary-length data - In this instance it returns error messages
        ID3DBlob* pErrorBlob;

        //Compiles Shader From FIle
        hr = D3DCompileFromFile(file_path, nullptr, nullptr, szEntryPoint, szShaderModel, dwShaderFlags, 0, ppBlobOut, &pErrorBlob);

        if (FAILED(hr))
        {
            if (pErrorBlob != nullptr)
            {
                OutputDebugStringA((char*)pErrorBlob->GetBufferPointer());
            }
            if (pErrorBlob)
            {
                pErrorBlob->Release();
            }
            return hr;
        }

        return S_OK;

    }
};
