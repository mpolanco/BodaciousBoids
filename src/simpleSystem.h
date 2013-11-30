#ifndef SIMPLESYSTEM_H
#define SIMPLESYSTEM_H

#include "extra.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <vecmath.h>

#include "particleSystem.h"

using namespace std;

class SimpleSystem: public ParticleSystem
{
public:
	SimpleSystem();
	
	vector<Vector3f> evalF(vector<Vector3f> state);
	
	void draw();
private:
    void loadDove();
    void drawDove();

    // Globals
    int MAX_BUFFER_SIZE = 100;
    // This is the list of points (3D vectors)
    vector<Vector3f> vecv;

    // This is the list of normals (also 3D vectors)
    vector<Vector3f> vecn;

    // This is the list of faces (indices into vecv and vecn)
    vector<vector<unsigned> > vecf;
	
};

#endif
