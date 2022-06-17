#pragma once
#include <d3d11_1.h>
#include <directxmath.h>

using namespace DirectX;


struct Lighting
{
	XMFLOAT4 AmbientLight;
	XMFLOAT4 DiffuseLight;
	XMFLOAT4 SpecularLight;
	FLOAT SpecularPower;
	XMFLOAT3 LightVecW;


};
