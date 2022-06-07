#pragma once
#include <d3d11_1.h>
#include <directxmath.h>

using namespace DirectX;

struct MeshData
{
	ID3D11Buffer* VertexBuffer;
	ID3D11Buffer* IndexBuffer;
	UINT VBStride;
	UINT VBOffset;
	UINT IndexCount;
};


