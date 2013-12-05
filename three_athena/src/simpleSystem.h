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

    Vector3f boxDims;
    float reboundFactor;
    float reboundZone;

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

    /* 
        BIRD VARIABLES AND METHODS 
    */
    int birdStartIndex; // the first index of birds in the state vector
    int m_numBirds;

    /* Bird Personality Traits: Orthogonal Basis
        daring: less emphasis on avoiding predators and obstacles
        social: smaller separation, more cohesion, larger neighborhood
        speedy: faster max speed
    */ 
    vector<float> daringness; // R
    vector<float> sociableness;  // B
    vector<float> speediness; // G

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
    Vector3f limitBirdVelocity(Vector3f vel, float speediness);
    // Returns a random float between 0 and 1
    float randf() { return ((float) rand()) / (float) RAND_MAX ;}
    float randf_sym() { return (((float) rand() * 2) / (float) RAND_MAX) - 1; }

    // Converts radians to degrees
    float rad_to_deg(float rad);

    /* 
        PREDATOR VARIABLES AND METHODS 
    */
        int predatorStartIndex; // the first index of predators in the state vector
        int m_numPredators;

        float predatorSeparation;
        float maxVelocityPredator;

        int findClosestPrey(vector<Vector3f> state, Vector3f predator_pos);
        Vector3f limitPredatorVelocity(Vector3f vel, float speediness);

    /* 
        OBSTACLE METHODS AND VARIABLES 
    */
        vector<Vector3f> sphereObstacles;
        vector<float> sphereObstacleRadius;
        float obstacleReboundZone;

        void drawObstacles();
        float randomObstacleSize();
        Vector3f avoidObstacles(Vector3f position, float daringness);

    /* 
        BOUNDING BOX METHODS AND VARIABLES 
    */
        void drawBoundingVertices();
        Vector3f boundPosition(Vector3f position);
        Vector3f randomPositionInBox();

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

    /* OTHER THINGS AND METHODS */
        float speedVariation;
        float getSpeediness(int bird_index);

        float getDaringness(int bird_index);

        float getSociableness(int bird_index);
        
};

#endif
