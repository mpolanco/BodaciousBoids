#ifndef CLOTHSYSTEM_H
#define CLOTHSYSTEM_H

#include "extra.h"
#include <vector>
#include <math.h>

#include "particleSystem.h"

class ClothSystem: public ParticleSystem
{
///ADD MORE FUNCTION AND FIELDS HERE
public:
	ClothSystem();
	vector<Vector3f> evalF(vector<Vector3f> state);
	
	void draw();

private:

	int CLOTH_WIDTH;
	int CLOTH_HEIGHT;
	float CLOTH_DETAIL;
	Vector3f windForce;

	Vector3f GRAVITY;
	float DRAG;
	std::vector< std::vector<Spring> > springTies;
	std::vector<float> masses;

	void initStructuralSprings();
	void initShearSprings();
	void initFlexSprings();
	void initMasses();
	int indexOf(int row, int col);
	void setSpring(int start, int end, float springConst, float restLength);

};


#endif
