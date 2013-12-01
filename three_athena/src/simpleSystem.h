#ifndef SIMPLESYSTEM_H
#define SIMPLESYSTEM_H

#include "extra.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <cstdlib>

#include "particleSystem.h"

using namespace std;

// Repurposed for Flock
class SimpleSystem: public ParticleSystem
{
public:
	SimpleSystem();
	
	vector<Vector3f> evalF(vector<Vector3f> state);
	
	void draw();

    Vector3f centerOfMass(vector<Vector3f> positions);

protected:
    float minSeparation;
    float minSquaredSeparation;
    float neighborCutoff;

    Vector3f perceivedCenter(Vector3f centerOfMass, Vector3f position);

    float randf() {
        return ((float) rand()) /  (float) RAND_MAX ;
    }
};

#endif
