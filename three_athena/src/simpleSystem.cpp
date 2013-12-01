
#include "simpleSystem.h"

using namespace std;

// Repurposed for Flock
SimpleSystem::SimpleSystem()
{
	m_numParticles = 10;
	for (int i=0; i < m_numParticles; i++) {
		Vector3f pos = Vector3f(i*0.4, randf(), randf());
		Vector3f vel = Vector3f(0, randf(), 0);

		m_vVecState.push_back(pos);
		m_vVecState.push_back(vel);
	}

	minSeparation = 0.5f;
	neighborCutoff = 2.0f;
	cout << "Init: articles should have minimum Separation: " << minSeparation << endl;

}

// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> SimpleSystem::evalF(vector<Vector3f> state)
{
	vector<Vector3f> f;
	Vector3f flockCenter = centerOfMass(state);
	// cout << "Flock center: " << flockCenter.x() << ", " << flockCenter.y() << ", " << flockCenter.z() << endl;

	// for each particle in the state
	// evaluate actual forces, except anchor the first particle
	for(int i=0; i < state.size(); i+=2) {
		// get the particles position and velocity
		Vector3f pos_i = state[i];
		Vector3f vel_i = state[i+1];

		Vector3f forces = Vector3f::ZERO;

		// Separation
		Vector3f sepForce = Vector3f::ZERO;
		float numTooClose = 0;
		// Alignment
		Vector3f alignForce = Vector3f::ZERO;
		float numNearby = 0;
		// Cohesion 
		Vector3f cohesiveForce = Vector3f::ZERO;

		for(int j=0; j < state.size(); j+=2) {
			Vector3f pos_j = state[j];
			Vector3f vel_j = state[j+1];

			if (i == j) {
				continue;
			}

			// Separation
			Vector3f diff = pos_i - pos_j;
			float dist = diff.abs();
			if ( dist < minSeparation ) {
				sepForce+= (diff) ;
				numTooClose++;
			}

			// Alignment
			if ( dist < neighborCutoff ) {
				alignForce+= vel_j;
				numNearby++;
			}

		}

		if (numTooClose > 0) { // avoid dividing by zero
			sepForce*= (1/numTooClose); // average the separation forces
		}

		if (numNearby > 0) { // avoid dividing by zero
			alignForce*= (1/numNearby); //average the align force
			alignForce-= vel_i; // Implement Reynolds: Change = Desired - Current
		}

		// Cohesion
		Vector3f perceived_center = perceivedCenter(flockCenter, pos_i);
		cohesiveForce = (perceived_center - pos_i) / 5; // weight the force so it's not overpowering
		
		forces+=sepForce;
		forces+=alignForce;
		forces+=cohesiveForce;		

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

Vector3f SimpleSystem::perceivedCenter(Vector3f centerOfMass, Vector3f position) 
{
	Vector3f perceived = centerOfMass - (position/m_numParticles);
	perceived*= m_numParticles/ (m_numParticles-1);
	return perceived;
}


// render the system (ie draw the particles)
void SimpleSystem::draw()
{
	Vector3f flockCenter = centerOfMass(m_vVecState);
	glPushMatrix();
	glTranslatef(flockCenter[0], flockCenter[1], flockCenter[2]);
	glutSolidSphere(0.075f,10.0f,10.0f);
	glPopMatrix();

	for (int i=0; i < m_numParticles; i+=2) {
		Vector3f pos = m_vVecState[i]; // PARTICLE POSITION
		Vector3f vel = m_vVecState[i+1]; // PARTICLE VELOCITY
		Vector3f top = -vel.normalized();

		glPushMatrix();
		glTranslatef(pos[0], pos[1], pos[2] );
		glEnable(GL_COLOR_MATERIAL);
		glColor3f(.2, .2, .1*i);
		glDisable(GL_COLOR_MATERIAL);
		glutSolidSphere(0.075f,10.0f,10.0f);
		glBegin(GL_TRIANGLES);
		glVertex(top);
		glVertex3f(-0.5f, -0.5f, -0.5f);
		glVertex3f(0.5f, 0.5f, 0.5f);
		glEnd();
		glPopMatrix();
	}
}
