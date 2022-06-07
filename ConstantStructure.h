#pragma once
#include <d3d11_1.h>
#include <directxmath.h>
#include "Materials.h"
#include "Lighting.h"

using namespace DirectX;

struct ConstantBuffer
{
	XMMATRIX World;
	XMMATRIX View;
	XMMATRIX Projection;
	Materials Mat;
	Lighting Light;
	XMFLOAT3 EyePosW;




};
