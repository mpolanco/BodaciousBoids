
#include "simpleSystem.h"

using namespace std;

// Repurposed for Flock
SimpleSystem::SimpleSystem()
{
	m_numParticles = 10;
	for (int i=0; i < m_numParticles; i++) {
		Vector3f pos = Vector3f(i*0.2, 0, 0);
		Vector3f vel = Vector3f(0, 0.2, 0);

		m_vVecState.push_back(pos);
		m_vVecState.push_back(vel);
	}

	float minSeparation = 0.5f;
	
}

// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> SimpleSystem::evalF(vector<Vector3f> state)
{
	vector<Vector3f> f;

	// for each particle in the state
	// evaluate actual forces, except anchor the first particle
	for(int i=0; i < state.size()/2; i++) {
		// get the particles position and velocity
		Vector3f pos_i = state[2*i];
		Vector3f vel_i = state[(2*i)+1];

		Vector3f forces = Vector3f::ZERO;
		// Separation

		// Alignment

		// Cohesion

		f.push_back(vel_i);
		f.push_back(forces);
	}

	return f;
}

Vector3f SimpleSystem::centerOfMass(vector<Vector3f> positions)
{
	return Vector3f::ZERO;
}

// render the system (ie draw the particles)
void SimpleSystem::draw()
{
	for (int i=0; i < m_numParticles; i++) {
		Vector3f pos = m_vVecState[2*i]; // PARTICLE POSITION

		glPushMatrix();
		glTranslatef(pos[0], pos[1], pos[2] );
		glutSolidSphere(0.075f,10.0f,10.0f);
		
		glPopMatrix();
	}
}
