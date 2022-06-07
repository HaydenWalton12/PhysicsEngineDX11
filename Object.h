#pragma once
#include "Render.h"
#include "Mesh.h"
#include "Transformation.h"
#include "Transformation.h"
#include "Surface.h"

#include "TextureComponent.h"
#include "OBJLoader.h"

#include "PixelShader.h"
#include "VertexShader.h"

struct Object
{
	char* Name;
	std::vector<ID3D11ShaderResourceView*> _Textures;
	MeshData _Mesh;
	Surface _ObjectMaterial;
	Transformation _ObjectTransformation;
	ID3D11VertexShader* _VS;
	ID3D11PixelShader* _PS;
	VertexShader _VertexShader;
	PixelShader _PixelShader;
	ID3D11RasterizerState* _RasterState;
	RenderCommands* _pRenderCommands;
	TextureComponent* _Tex;
	Object() = default;

	Object(RenderCommands* render_commands, wchar_t* texture, TextureComponent* Tex, char* object)
	{
		_Tex = Tex;
		_pRenderCommands = render_commands;
		Name = object;
		CreateTexture(texture);
		_Mesh = OBJLoader::Load(object, render_commands->GetDevice(), false);
	}
	Object(RenderCommands* render_commands, char* object)
	{
		_Tex = nullptr;
		_pRenderCommands = render_commands;
		Name = object;
		_Mesh = OBJLoader::Load(object, render_commands->GetDevice(), false);
	}
	XMFLOAT3 GetTranslation()
	{
		return _ObjectTransformation.GetTranslate();
	}

	void SetTransformation(XMFLOAT3 translation, XMFLOAT3 scale, XMFLOAT3 rotation)
	{
		_ObjectTransformation = Transformation(translation, scale, rotation);
	}
	void SetTranslation(XMFLOAT3 translation)
	{
		_ObjectTransformation.SetTranslation(translation);
	}
	void SetRotation(XMFLOAT3 rotation)
	{
		_ObjectTransformation.SetRotation(rotation);
	}
	void SetRotation(XMMATRIX rotation)
	{
		_ObjectTransformation.SetRotation(rotation);
	}
	void SetScale(XMFLOAT3 scale)
	{
		_ObjectTransformation.SetScale(scale);
	}

	void SetTransformation1(XMFLOAT3 translation)
	{
		_ObjectTransformation.SetTranslation(translation);
	}

	void SetTransformation2(XMFLOAT3 translation, XMFLOAT3 scale, XMFLOAT3 rotation)
	{
		_ObjectTransformation = Transformation(translation, scale, rotation);
	}

	void SetSurface(XMFLOAT4 ambient, XMFLOAT4 diffuse, XMFLOAT4 specular, FLOAT specular_power)
	{
		_ObjectMaterial = Surface(ambient, diffuse, specular, specular_power);
	}

	void SetPixelShader(WCHAR* file_path)
	{
		_PixelShader = PixelShader(file_path, _pRenderCommands->GetDevice());
	}
	void SetVertexShader(WCHAR* file_path)
	{
		_VertexShader = VertexShader(file_path, _pRenderCommands->GetDevice(), _pRenderCommands->GetDeviceContext());
	}


	void Draw(Camera* camera)
	{
		UINT stride = sizeof(SimpleVertex);
		UINT offset = 0;

		_pRenderCommands->BindVertexShader(_VertexShader.GetVertexShader());
		_pRenderCommands->BindPixelShader(_PixelShader.GetPixelShader());
		_pRenderCommands->UpdateConstantBuffer(camera, _ObjectTransformation.GetWorld(), _ObjectMaterial);
		_pRenderCommands->GetDeviceContext()->IASetVertexBuffers(0, 1, &_Mesh.VertexBuffer, &stride, &offset);
		_pRenderCommands->GetDeviceContext()->IASetIndexBuffer(_Mesh.IndexBuffer, DXGI_FORMAT_R16_UINT, 0);
		if (_Tex != nullptr)
		{
			_Tex->BindTextures(0, _Textures.size(), _Textures, _pRenderCommands);

		}

		_pRenderCommands->GetDeviceContext()->DrawIndexed(_Mesh.IndexCount, 0, 0);

	}

	void Update()
	{



	}

	void CreateTexture(wchar_t* path)
	{
		ID3D11ShaderResourceView* texture;
		_Tex->CreateTexture(path, &texture, _pRenderCommands);
		_Textures.push_back(texture);
	}





	MeshData GetMesh()
	{
		return _Mesh;
	}
};