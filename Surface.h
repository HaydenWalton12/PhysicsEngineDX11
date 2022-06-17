#pragma once
#include <d3d11_1.h>
#include <directxmath.h>

using namespace DirectX;


struct Surface
{
	XMFLOAT4 Ambient;
	XMFLOAT4 Diffuse;
	XMFLOAT4 Specular;

	FLOAT SpecularPower;

	Surface() = default;

	Surface(XMFLOAT4 ambient, XMFLOAT4 diffuse, XMFLOAT4 specular, FLOAT specular_power)
	{
		Ambient = ambient;
		Diffuse = diffuse;
		Specular = specular;
		SpecularPower = specular_power;
	}

	Surface GetSurface()
	{
		return*this;
	}

	XMFLOAT4 GetAmbient()
	{
		return Ambient;
	}
	XMFLOAT4 GetDiffuse()
	{
		return Diffuse;
	}
	XMFLOAT4 GetSpecular()
	{
		return Specular;
	}

};