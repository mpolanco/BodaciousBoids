#ifndef PARTICLESYSTEM_H
#define PARTICLESYSTEM_H

#include <vector>
#include <vecmath.h>
#include <iostream>

using namespace std;

class ParticleSystem
{
public:

	ParticleSystem(int numParticles=0 );

	int m_numParticles;
	bool areSpringsVisible;
	bool areParticlesVisible;
	bool wind;
	
	// for a given state, evaluate derivative f(X,t)
	virtual vector<Vector3f> evalF(vector<Vector3f> state) = 0;
	
	// getter method for the system's state
	vector<Vector3f> getState(){ return m_vVecState; };

	virtual int getRandomBirdPositionIndex() = 0;

	virtual Vector3f getPositionAtIndex(int ind) = 0;

	//toggles spring visibility
	void toggleSprings(){ areSpringsVisible = !areSpringsVisible; };
	void toggleParticles(){ areParticlesVisible = !areParticlesVisible; };	
	void toggleWind(){ wind = !wind; };	

	void shiftRoot(Vector3f delta){rootDelta = delta;};
	
	// setter method for the system's state
	void setState(const vector<Vector3f>  & newState) { m_vVecState = newState; };

	void step() { time_step++; }
	
	virtual void draw() = 0;
	
protected:

	vector<Vector3f> m_vVecState;
	Vector3f rootDelta;
	int time_step;
	float DRAG_CONSTANT;

	Vector3f evalViscousDrag(Vector3f vel);
};

#endif

#ifndef SPRING_H
#define SPRING_H

class Spring
{
public:
	/* data */
	int particleInd1;
	int particleInd2;

	float k;
	float restLength;

	inline Spring(int i, int j, float spConst, float rlen) {
		particleInd1 = i;
		particleInd2 = j;
		k = spConst;
		restLength = rlen;
	}

};

#endif
