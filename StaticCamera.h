#pragma once

#include "CameraComponent.h"
class StaticDefaultCamera
{
public:
	StaticDefaultCamera()
	{

		_Camera_Position = XMFLOAT3(0.0f, 0.0f, -10.0f);
		_Camera_Direction = XMFLOAT3(0.0f, 0.0f, 0.1f);
		_Camera = Camera(_Camera_Position, _Camera_Direction);
	}


	void UpdateCamera()
	{
		_Camera.UpdateCamera();
	}

	Camera _Camera;

private:

	XMFLOAT3 _Camera_Position;
	XMFLOAT3 _Camera_Direction;
};
