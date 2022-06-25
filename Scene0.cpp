#include "Scene0.h"
#include "Collision.h"
#include "BroadPhase.h"

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

//The Engine Handles All Forces As Impulses , Essentially Applying These Impulse Forces Whenever Needed, From this , we need to further know what
//each impusle is and how they affect momentum.
//Momentum is the product of mass and velocity:
// Momentum = Mass * Velocity - p = m * v
// Momentum can be defined as the amount a moving ovject would like to keep moving in a direction , the more momentum an object has, 
//the more force required to change its momentum , the change in momentum is related to the applied force using :
// DT - Time Derivative - Expresses the rate of change of a value over time
// 
// dp = F * dt

//Impulse is defined by the change of momentum , this makes 
//sense since depending on the force applied to increase the momentum , this would
//be considered as an impulse

//Impulses 



void Scene0::Update(float delta_time) 
{
	DrawUI();
	_SceneCamera->UpdateCamera();

	//Gravity - All objects are and will be effected by gravity, a simple method of moving. Positions change when velocity value increases or decreases.
	//Velocity - Rate of speed of an object in a given direction, direction represented Vector3 , change of position within velocity is 
	// - dx = v ∗ dt
	// Distance =  velocity * distance / time

	//Velocity changes with acceleration , equation relating the change of velocity within acceleration is
	// dv = a ∗ dt
	// change of velocity = acceleration * distance / time

	//However we set gravity as an impulse , since gravity is a force, forces act on bodies that have mass.
	//Forces are defined as - F (Force) * Mass * Acceleration

	//When Applying the force of gravity , when applied near the surface of earth , gravity is defined as :
	// F = Mass * Gravity (F = M * G)
	// Force = Mass * Gravity 



	for (int i = 0; i < _SceneBodies.size(); i++)
	{
		Body* body = &_SceneBodies[i];
		float mass = 1.0f / body->_InvMass;

		//Remember , we apply all forces as an impulse
		// J = dp (change in momentum) , Force = change in momentum / change in time 
		//We calculate gravity impusle applying the gravity to given mass, this will give us the impusle value , but also give us a weight value (W = M * G)
		Vec3 impulse_gravity = body->_Gravity * mass * delta_time;

		//Apply the impulse, changes the momentum
		body->AddImpulseLinear(impulse_gravity);
	}


	//BroadPhase
	std::vector<CollisionPair> collision_pairs;
	BroadPhase(_SceneBodies.data(), (int)_SceneBodies.size(), collision_pairs, delta_time);

	//NarrowPhase - Actual Collision Detection
	int num_contact = 0;
	const int max_contacts = _SceneBodies.size() * _SceneBodies.size();
	Contact* contacts = (Contact*)alloca(sizeof(Contact) * max_contacts);

	for (int i = 0; i < collision_pairs.size(); i++)
	{
		const CollisionPair& pair = collision_pairs[i];
		Body* bodyA = &_SceneBodies[pair.a];
		Body* bodyB = &_SceneBodies[pair.b];

		if (0.0f == bodyA->_InvMass && 0.0f == bodyB->_InvMass)
		{
			continue;
		}

		Contact contact;
		if (Collision::Intersect(bodyA, bodyB ,delta_time , contact))
		{
			contacts[num_contact] = contact;
			num_contact++;
		}
	}
	//Sort Contacts , times of impacts first to last
	if (num_contact > 1)
	{
		qsort(contacts, num_contact, sizeof(Contact), CompareContacts);
	}


	//Apply ballistic impuslses
	float accumTime = 0.0f;
	for (int i = 0; i < num_contact; i++)
	{
		Contact& contact = contacts[i];
		const float dt = contact._TimeOfImpact - accumTime;
		//position update
		for (int j = 0; j < _SceneBodies.size(); j++)
		{
			_SceneBodies[j].Update(delta_time);

		}
		ResolveContacts(contact);
		accumTime += dt;
	}

	//Update Positions for rest of frames time
	const float time_remaining = delta_time - accumTime;
	if (time_remaining > 0.0f)
	{
		for (int j = 0; j < _SceneBodies.size(); j++)
		{
			_SceneBodies[j].Update(time_remaining );

			_SceneBodies[j]._Shape->_Object->_ObjectTransformation.SetTranslation(XMFLOAT3(_SceneBodies[j]._Position.x, _SceneBodies[j]._Position.y, _SceneBodies[j]._Position.z));
			_SceneBodies[j]._Shape->_Object->_ObjectTransformation.SetRotation(XMFLOAT3(_SceneBodies[j]._AngularVelocity.x, _SceneBodies[j]._AngularVelocity.y, _SceneBodies[j]._AngularVelocity.z));
			_SceneBodies[j]._Shape->_Object->_ObjectTransformation.UpdateObject();
			_SceneBodies[j]._Shape->_Object->Draw(_SceneCamera);
		}

	}


}