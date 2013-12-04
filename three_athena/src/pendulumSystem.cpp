#include <iostream>
#include "pendulumSystem.h"

PendulumSystem::PendulumSystem(int numParticles):ParticleSystem(numParticles)
{

	m_vVecState.push_back(Vector3f(0, 0, 0));
	m_vVecState.push_back(Vector3f(0, 0, 0));
	
	// fill in code for initializing the state based on the number of particles
	for (int i = 0; i < m_numParticles; i++) {
		
		// for this system, we care about the position and the velocity
		
		m_vVecState.push_back(Vector3f( 1.0 * (i+1), 0 ,0));
		m_vVecState.push_back(Vector3f(1, 0, 0));
	}

}

// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> PendulumSystem::evalF(vector<Vector3f> state)
{
	vector<Vector3f> f;
	// YOUR CODE HERE
	for (int i = 0; i < state.size(); i += 2)
	{
		
	}

	return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw()
{
	for (int i = 0; i < m_numParticles; i++) {
		Vector3f pos = m_vVecState[i*2];//  position of particle i. YOUR CODE HERE

		if (areParticlesVisible)
		{
			glPushMatrix();
			glTranslatef(pos[0], pos[1], pos[2] );
			glutSolidSphere(0.075f,10.0f,10.0f);
			
			glPopMatrix();
		}

	
	}
}
