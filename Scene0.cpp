#include "Scene0.h"


int CompareContacts(const void* p1, const void* p2) {
	const Contact a = *(const Contact*)p1;
	const Contact b = *(const Contact*)p2;

	if (a._TimeOfImpact < b._TimeOfImpact) {
		return -1;
	}

	if (a._TimeOfImpact == b._TimeOfImpact) {
		return 0;
	}

	return 1;
}

void Scene0::Update(float delta_time) 
{
	DrawUI();
	_SceneCamera->UpdateCamera();



	//Gravity
	for (int i = 0; i < _SceneBodies.size(); i++)
	{
		Body* body = &_SceneBodies[i];
		float mass = 1.0f / body->_InvMass;
		Vec3 impulse_gravity = Vec3(0.0f, -10.0f, 0.0f) * mass * delta_time;
		body->AddImpulseLinear(impulse_gravity);
	}

	int num_contact = 0;
	const int max_contacts = _SceneBodies.size() * _SceneBodies.size();

	Contact* contacts = (Contact*)alloca(sizeof(Contact) * max_contacts);

	for (int i = 0; i < _SceneBodies.size(); i++)
	{
		for (int j = i + 1; j < _SceneBodies.size(); j++)
		{
			Body* bodyA = &_SceneBodies[i];
			Body* bodyB = &_SceneBodies[j];

			//Skip If Inf Mass
			if (0.0f == bodyA->_InvMass && 0.0f == bodyB->_InvMass)
			{
				continue;
			}
			Contact contact;
			if (Intersect(bodyA, bodyB, delta_time, contact))
			{
				contacts[num_contact] = contact;
				num_contact++;
			}
		}
	}
	if (num_contact > 1)
	{
		qsort(contacts, num_contact, sizeof(Contact), CompareContacts);
	}

	float accumTime = 0.0f;
	for (int i = 0; i < num_contact; i++)
	{
		Contact& contact = contacts[i];
		const float dt = contact._TimeOfImpact - accumTime;

		Body* bodyA = contact._BodyA;
		Body* bodyB = contact._BodyB;

		//Skip If Inf Mass
		if (0.0f == bodyA->_InvMass && 0.0f == bodyB->_InvMass)
		{
			continue;
		}
		for (int j = 0; j < _SceneBodies.size(); j++)
		{
			_SceneBodies[j].Update(delta_time);
	
		}

		ResolveContacts(contact);
		accumTime += dt;


	}

	const float time_remaining = delta_time - accumTime;
	if (time_remaining > 0.0f)
	{
		for (int j = 0; j < _SceneBodies.size(); j++)
		{
			_SceneBodies[j].Update(time_remaining );
		}

	}

	for (int j = 0; j < _SceneBodies.size(); j++)
	{
		
		_SceneBodies[j]._Shape->_Object->_ObjectTransformation.SetTranslation(XMFLOAT3(_SceneBodies[j]._Position.x, _SceneBodies[j]._Position.y, _SceneBodies[j]._Position.z));
		_SceneBodies[j]._Shape->_Object->_ObjectTransformation.SetRotation(XMFLOAT3(_SceneBodies[j]._AngularVelocity.x, _SceneBodies[j]._AngularVelocity.y, _SceneBodies[j]._AngularVelocity.z));
		_SceneBodies[j]._Shape->_Object->_ObjectTransformation.UpdateObject();
		_SceneBodies[j]._Shape->_Object->Draw(_SceneCamera);
	}
}