#ifndef PENDULUMSYSTEM_H
#define PENDULUMSYSTEM_H

#include "extra.h"
#include <vector>

#include "particleSystem.h"

class PendulumSystem: public ParticleSystem
{
public:
	PendulumSystem(int numParticles);
	
	vector<Vector3f> evalF(vector<Vector3f> state);
	
	void draw();

private:
	Vector3f GRAVITY;
	float DRAG;
	
	void initSpringTies();
	void initMasses();

	std::vector< std::vector<Spring> > springTies;
	std::vector<float> masses;
};

#endif
