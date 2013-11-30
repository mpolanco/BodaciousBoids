#ifndef CLOTHSYSTEM_H
#define CLOTHSYSTEM_H

#include "extra.h"
#include <vector>
#include <math.h>

#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <vecmath.h>

#include "particleSystem.h"

class ClothSystem: public ParticleSystem
{
///ADD MORE FUNCTION AND FIELDS HERE
public:
	ClothSystem();
	vector<Vector3f> evalF(vector<Vector3f> state);
	
	void draw();

private:

	const int CLOTH_WIDTH = 8;
	const int CLOTH_HEIGHT = 8;
	const float CLOTH_DETAIL = .5;
	const Vector3f windForce = Vector3f(0,0,-5);

	Vector3f GRAVITY = Vector3f(0.0,-9.8,0.0);
	float DRAG = .5;
	std::vector< std::vector<Spring> > springTies;
	std::vector<float> masses;

	void initStructuralSprings();
	void initShearSprings();
	void initFlexSprings();
	void initMasses();
	int indexOf(int row, int col);
	void setSpring(int start, int end, float springConst, float restLength);


	//Testing dove drawing
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
