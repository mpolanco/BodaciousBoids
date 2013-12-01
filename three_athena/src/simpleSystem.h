#ifndef SIMPLESYSTEM_H
#define SIMPLESYSTEM_H

#include "extra.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
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
    float neighborCutoff;

    Vector3f perceivedCenter(Vector3f centerOfMass, Vector3f position);

    float randf() {
        return ((float) rand()) /  (float) RAND_MAX ;
    }

    float rad_to_deg(float rad);

    //Testing dove drawing
    void loadDove();
    void drawDove();

    // Globals
    int MAX_BUFFER_SIZE;
    // This is the list of points (3D vectors)
    vector<Vector3f> vecv;

    // This is the list of normals (also 3D vectors)
    vector<Vector3f> vecn;

    // This is the list of faces (indices into vecv and vecn)
    vector<vector<unsigned> > vecf;
};

#endif
