#include <iostream>
#include "pendulumSystem.h"

PendulumSystem::PendulumSystem(int numParticles):ParticleSystem(numParticles)
{
	GRAVITY = Vector3f(0.0,-9.8,0.0);
	DRAG = .5;

	m_numParticles = numParticles;

	m_vVecState.push_back(Vector3f(0, 0, 0));
	m_vVecState.push_back(Vector3f(0, 0, 0));
	
	// fill in code for initializing the state based on the number of particles
	for (int i = 0; i < m_numParticles-1; i++) {
		
		// for this system, we care about the position and the velocity
		
		m_vVecState.push_back(Vector3f( 1.0 * (i+1), 0 ,0));
		m_vVecState.push_back(Vector3f(1, 0, 0));
	}

	initMasses();

	//define spring relations
	initSpringTies();
}

void PendulumSystem::initMasses()
{
	for (int i = 0; i < m_numParticles; ++i)
	{
		masses.push_back( 1.0 );
	}
}

void PendulumSystem::initSpringTies()
{
	float restLength = 0.1;
	float k = 40.0;

	std::vector<Spring> root_springs;

	Spring spring = Spring(0, 2, k, restLength);
	root_springs.push_back(spring);
	springTies.push_back(root_springs);

	for (int i = 1; i < m_numParticles-1; ++i)
	{
		std::vector<Spring> v;
		Spring prev = Spring(i*2, (i-1) * 2, k, restLength);
		Spring next = Spring(i*2, (i+1) * 2, k, restLength);
		v.push_back(prev);
		v.push_back(next);
		springTies.push_back(v);
	}

	std::vector<Spring> final_springs;
	Spring final_spring = Spring((m_numParticles-1) * 2, (m_numParticles-2) * 2, k, restLength);
	final_springs.push_back(final_spring);
	springTies.push_back(final_springs);
}


// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> PendulumSystem::evalF(vector<Vector3f> state)
{
	vector<Vector3f> f;
	// YOUR CODE HERE
	for (int i = 0; i < state.size(); i += 2)
	{
		//get particle mass 
		float mass = masses[i/2];

		//get position and velocity of particle
		Vector3f particlePos = state[i];
		Vector3f particleVel = state[i+1];
		Vector3f particleForce = Vector3f(0.0, 0.0, 0.0);
		
		//calculate forces
		//gravitational force
		particleForce += mass * GRAVITY;

		//drag
		particleForce += mass * -1.0 * DRAG * particleVel;

		//spring force
		std::vector<Spring> particleSprings = springTies[i/2];
		for (int k = 0; k < particleSprings.size(); ++k)
		{
			Spring particleSpring = particleSprings[k];

			Vector3f pos1 = state[particleSpring.particleInd1];
			Vector3f pos2 = state[particleSpring.particleInd2];

			Vector3f dist = pos1 - pos2;

			Vector3f spForce = -1.0 * particleSpring.k * (dist.abs() - particleSpring.restLength) * (dist/dist.abs());

			particleForce += mass * spForce;
		}

		//push dx (velocity)
		f.push_back(particleVel);
		//push dv (acceleration)
		f.push_back(particleForce/mass);
	}

	//keep initial particle at rest
	f[0] = rootVel;
	f[1] = Vector3f(0,0,0);

	shiftRoot(Vector3f::ZERO);

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

		if (areSpringsVisible)
		{
			std::vector<Spring> springs = springTies[i];
			for (int k = 0; k < springs.size(); ++k)
			{
				glBegin(GL_LINES);
				glVertex(pos);
				glVertex(m_vVecState[springs[k].particleInd2]);
				glEnd();
			}
		}
	}
}
