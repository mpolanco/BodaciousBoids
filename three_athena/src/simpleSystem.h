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

	SimpleSystem(int numBirds, int numPredators);
	
    int GOAL_DEFAULT ;
    int GOAL_CIRCULAR;
    int GOAL_ZIGZAG;

    int xDim;
    int yDim;
    int zDim;

    int box_size;


	vector<Vector3f> evalF(vector<Vector3f> state);

    Vector3f getPositionAtIndex(int ind);
    int getRandomBirdPositionIndex();
	
	void draw();

    Vector3f centerOfMass(vector<Vector3f> positions);
    void shiftRoot(Vector3f delta);
    void step();
    void setGoalPattern(int patternId);

protected:
    void updateGoal(int time_step);

    /* BIRD VARIABLES AND METHODS */
    int birdStartIndex; // the first index of birds in the state vector
    int m_numBirds;

    float minSeparation;
    float neighborCutoff;
    float maxVelocityBird;

    float cohesiveWeight;
    float alignWeight;
    float separationWeight;
    float goalWeight;

    Vector3f m_goalPos;
    Vector3f m_goalVel;
    int goalPatternId;

    Vector3f perceivedCenter(Vector3f centerOfMass, Vector3f position);
    Vector3f limitBirdVelocity(Vector3f vel);
    // Returns a random float between 0 and 1
    float randf() { return ((float) rand()) / (float) RAND_MAX ;}

    // Converts radians to degrees
    float rad_to_deg(float rad);

    /* PREDATOR VARIABLES AND METHODS */
    int predatorStartIndex; // the first index of predators in the state vector
    int m_numPredators;

    float predatorSeparation;
    float maxVelocityPredator;

    int findClosestPrey(vector<Vector3f> state, Vector3f predator_pos);
    Vector3f limitPredatorVelocity(Vector3f vel);

    /* 
        DOVE METHODS AND VARIABLES 
    */
        void loadDove();
        void drawDove();

        // Globals for loading dove
        int MAX_BUFFER_SIZE;
        // This is the list of points (3D vectors)
        vector<Vector3f> vecv;
        // This is the list of normals (also 3D vectors)
        vector<Vector3f> vecn;
        // This is the list of faces (indices into vecv and vecn)
        vector<vector<unsigned> > vecf;

    /* OTHER METHODS */
        void drawObstacles();
        void drawBoundingVertices();

};

#endif
