
#include "simpleSystem.h"

using namespace std;

// Repurposed for Flock
SimpleSystem::SimpleSystem()
{
	m_numParticles = 10;
	for (int i=0; i < m_numParticles; i++) {
		Vector3f pos = Vector3f(i*0.45, 0, 0);
		Vector3f vel = Vector3f(0, 0.2, 0);

		m_vVecState.push_back(pos);
		m_vVecState.push_back(vel);
	}

	minSeparation = 0.5f;
	minSquaredSeparation = pow(minSeparation, 2);
	cout << "Init: articles should have minimum Separation: " << minSeparation << endl;
	cout << "Init: And minSquaredSeparation: " << minSquaredSeparation << endl;

}

// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> SimpleSystem::evalF(vector<Vector3f> state)
{
	vector<Vector3f> f;

	// for each particle in the state
	// evaluate actual forces, except anchor the first particle
	for(int i=0; i < state.size(); i+=2) {
		// get the particles position and velocity
		Vector3f pos_i = state[i];
		Vector3f vel_i = state[i+1];

		Vector3f forces = Vector3f::ZERO;

		// Separation
		for(int j=0; j < state.size(); j+=2) {
			Vector3f pos_j = state[j];
			if (i == j) {
				continue;
			}
			Vector3f diff = pos_i - pos_j;
			if ( diff.absSquared() < minSquaredSeparation ) {
				forces-= diff;
			}
		}

		// Alignment

		// Cohesion

		f.push_back(vel_i);
		f.push_back(forces);
	}

	return f;
}

Vector3f SimpleSystem::centerOfMass(vector<Vector3f> state)
{
	Vector3f summedPos = Vector3f::ZERO;
	float numPos = state.size()/2;
	for(int i=0; i < state.size(); i+=2) {
		summedPos+= state[i];
	}
	return summedPos/numPos;
}


// render the system (ie draw the particles)
void SimpleSystem::draw()
{
	for (int i=0; i < m_numParticles; i++) {
		Vector3f pos = m_vVecState[2*i]; // PARTICLE POSITION

		glPushMatrix();
		glTranslatef(pos[0], pos[1], pos[2] );
		glEnable(GL_COLOR_MATERIAL);
		glColor3f(.2, .2, .1*i);
		glDisable(GL_COLOR_MATERIAL);
		glutSolidSphere(0.075f,10.0f,10.0f);
		
		glPopMatrix();
	}
}
