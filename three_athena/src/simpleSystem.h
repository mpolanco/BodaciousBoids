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

    bool showFeathers;

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

    void toggleShowFeathers() {
        showFeathers = !showFeathers;
    }

protected:
    void updateGoal(int time_step);

    /* 
        BIRD VARIABLES AND METHODS 
    */
    int birdStartIndex; // the first index of birds in the state vector
    int m_numBirds;

    vector<Vector3f> doveColors;
    vector<Vector3f> eagleColors;

    /* Bird Personality Traits: Orthogonal Basis
        daring: less emphasis on avoiding predators and obstacles
        social: smaller separation, more cohesion
        speedy: faster max speed
    */ 
    vector<float> daringness; // R
    vector<float> sociableness;  // B
    vector<float> speediness; // G
    vector<int> colorIndex;

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

        void populateDoveColors() {
            // doveColors.push_back(Vector3f(0.98, 0.98, 0.82)); // lemon chiffon
            // doveColors.push_back(Vector3f(1.0, .97, .86)); // corn silk
            // doveColors.push_back(Vector3f(.96, .87, 0.7 )); // wheat
            // doveColors.push_back(Vector3f(.98, .92, .84)); // antique white
            doveColors.push_back(Vector3f(1, .89, .71)); // moccasin
            // doveColors.push_back(Vector3f(1, .89, .88));
            // doveColors.push_back(Vector3f(.96, 1, .98));

            doveColors.push_back(Vector3f(1, 0.75, 0.79)); // pink
            doveColors.push_back(Vector3f(.87, 0.72, 0.53)); // burly wood
            doveColors.push_back(Vector3f(.96, 0.64, 0.38)); // sandy brown
            doveColors.push_back(Vector3f(1.0, 0.5, 0.31)); // coral

        }

        void populateEagleColors() {
            eagleColors.push_back(Vector3f(0.545, 0.27, 0.074)); // saddle brown
            eagleColors.push_back(Vector3f(0.63, .32, .18)); // sienna
        }

        int getRandomDoveColorIndex() {
            return (int) (randf() * doveColors.size());
        }

        int getRandomDoveColorIndex(int i) {
            return i % doveColors.size();
        }
        
        int getRandomEagleColorIndex() {
            return (int) (randf() * eagleColors.size());
        }

        Vector3f getFeatherColor(int bird_index) {
            int ind = (bird_index - birdStartIndex) / 2;
            int colInd = colorIndex[ind];
            if (ind >= m_numBirds) {
                return eagleColors[colInd];
            }
            return doveColors[colInd];
        }

        Vector3f getPersonalityColor(int bird_index) {
            float dar = getDaringness(bird_index);
            float soc = getSociableness(bird_index);
            float spd = getSpeediness(bird_index);
            return Vector3f(dar, 0.3 + 0.3*spd, soc);
        }

        Vector3f getBirdColor(int bird_index) {
            if (showFeathers){ 
                return getFeatherColor(bird_index); 
            }
            return getPersonalityColor(bird_index);
        }

};

#endif
