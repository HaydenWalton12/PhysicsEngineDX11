#pragma once
#include <d3d11_1.h>
#include <directxmath.h>


using namespace DirectX;



struct Camera
{
	FLOAT _WindowWidth;
	FLOAT _WindowHeight;

	XMFLOAT3 _Camera_Position;
	XMFLOAT3 _Camera_Direction;

	XMFLOAT4X4 _View;
	XMFLOAT4X4 _Projection;

	FLOAT _NearDepth;
	FLOAT _FarDepth;

	Camera() = default;

	Camera(XMFLOAT3 camera_position, XMFLOAT3 camera_direction)
	{

		_Camera_Position = camera_position;
		_Camera_Direction = camera_direction;
		_NearDepth = 0.01f;
		_FarDepth = 100.0f;
		_WindowHeight = 640;
		_WindowWidth = 480;


		XMVECTOR Direction, Position, Up;
		Position = XMVectorSet(camera_position.x, camera_position.y, camera_position.z, 0.0f);
		Direction = XMVectorSet(camera_direction.x, camera_direction.y, camera_direction.z, 0.0f);
		Up = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);

		//Initalize view matrix
		XMStoreFloat4x4(&_View, XMMatrixLookToLH(Position, Direction, Up));

		XMStoreFloat4x4(&_Projection, XMMatrixPerspectiveFovLH(XM_PIDIV2, _WindowWidth / (FLOAT)_WindowHeight, _NearDepth, _FarDepth));

	}

	void UpdateCamera()
	{
		XMVECTOR Direction, Position, Up;
		Position = XMVectorSet(_Camera_Position.x, _Camera_Position.y, _Camera_Position.z, 0.0f);
		Direction = XMVectorSet(_Camera_Direction.x, _Camera_Direction.y, _Camera_Direction.z, 0.0f);
		Up = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);

		//Initalize view matrix
		XMStoreFloat4x4(&_View, XMMatrixLookToLH(Position, Direction, Up));



	}

#pragma region Getters

	XMFLOAT4X4 GetView() const { return _View; }
	XMFLOAT4X4 GetProjection() const { return _Projection; }
	XMFLOAT3 GetPosition() const { return _Camera_Position; }
	XMFLOAT3 GetDirection() const { return _Camera_Direction; }


#pragma endregion

};
