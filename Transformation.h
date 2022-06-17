#pragma once
#include <d3d11_1.h>
#include <directxmath.h>

using namespace DirectX;



struct Transformation
{
	XMFLOAT4X4 World;
	XMFLOAT3 Translation;
	XMFLOAT3 Scale;
	XMFLOAT3 Rotation;
	Transformation() = default;


	XMMATRIX _Translation;
	XMMATRIX _Scale;
	XMMATRIX _Rotation;

	Transformation(XMFLOAT3 translation, XMFLOAT3 scale, XMFLOAT3 rotation)
	{
		Translation = translation;
		Scale = scale;
		Rotation = rotation;

		_Translation = XMMatrixTranslation(Translation.x, Translation.y, Translation.z);
		_Scale = XMMatrixScaling(Scale.x, Scale.y, Scale.z);
		_Rotation = XMMatrixRotationRollPitchYaw(Rotation.x, Rotation.y, Rotation.z);

	}



	XMFLOAT4X4 GetWorld()
	{
		return World;
	}

	XMFLOAT3 GetRotation()
	{
		return Rotation;

	}

	XMFLOAT3 GetScale()
	{
		return Scale;

	}

	XMFLOAT3 GetTranslate()
	{
		return Translation;

	}

	void SetRotation(XMFLOAT3 rotation)
	{
		_Rotation = XMMatrixRotationRollPitchYaw(rotation.x, rotation.y, rotation.z);
		Rotation = rotation;

	}
	void SetRotation(XMMATRIX rotation)
	{
		_Rotation = rotation;
	}

	void SetTranslation(XMFLOAT3 translation)
	{

		_Translation = XMMatrixTranslation(translation.x, translation.y, translation.z);
		Translation = translation;

	}

	void UpdateObject()
	{

		XMStoreFloat4x4(&World, _Scale * _Rotation * _Translation);

	}
	void SetScale(XMFLOAT3 scale)
	{
		_Scale = XMMatrixScaling(scale.x, scale.y, scale.z);
		Scale = scale;


	}
};
