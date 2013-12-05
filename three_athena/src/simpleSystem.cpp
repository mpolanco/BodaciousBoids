
#include "simpleSystem.h"

using namespace std;

// Repurposed for Flock

SimpleSystem::SimpleSystem(int numBirds, int numPredators)
{
    GOAL_DEFAULT = 0;
    GOAL_CIRCULAR = 1;
    GOAL_ZIGZAG = 2;

    boxDims = Vector3f(4,4,4);
    reboundFactor = 2.0f;
    reboundZone = 0.1f;
    cout << "Bounding box X, Y, Z: " << boxDims.x() << ", " << boxDims.y() << ", " << boxDims.z() << endl;

	areParticlesVisible = false;
	m_numParticles = numBirds + numPredators;
    m_numBirds = numBirds;
    m_numPredators = numPredators;

    // push the goal's pos and vel first before any birds
    goalPatternId = 0;
    m_goalPos = Vector3f(0,0,0);
    m_goalVel = Vector3f(0,0,0);
    m_vVecState.push_back(m_goalPos);
    m_vVecState.push_back(m_goalVel);

    birdStartIndex = 2;
	for (int i=0; i < numBirds; i++) {
		Vector3f pos = Vector3f(i*0.4, randf(), randf());
		Vector3f vel = Vector3f(0, randf(), 0);

		m_vVecState.push_back(pos);
		m_vVecState.push_back(vel);
        daringness.push_back(randf_sym());
        sociableness.push_back(randf_sym());
        speediness.push_back(randf_sym());
	}

    predatorStartIndex = m_vVecState.size();
    for (int j=0; j < numPredators; j++) {
        Vector3f pos = Vector3f(-randf(), 1, 1);
        Vector3f vel = Vector3f(randf(), randf(), randf());

        m_vVecState.push_back(pos);
        m_vVecState.push_back(vel);
        daringness.push_back(randf_sym());
        sociableness.push_back(randf_sym());
        speediness.push_back(randf_sym());
    }

    cohesiveWeight = 1 / 4.0f;
    alignWeight = (1.0/8.0f);
    separationWeight = 1.0f;
    goalWeight = 1.0f;

	minSeparation = 1.0f;
	neighborCutoff = 3.0f;

    maxVelocityBird = 1.2f;
    maxVelocityPredator = 0.8f * maxVelocityBird;
    speedVariation = 0.4;

    predatorSeparation = 2.5f;
	cout << "Init: articles should have minimum Separation: " << minSeparation << endl;
	MAX_BUFFER_SIZE = 100;
	loadDove();

    /* Initialize Random obstacles */
    obstacleReboundZone = 0.1;
    int num_sphere_obstacles = 10;
    for (int i=0; i<num_sphere_obstacles; i++) {
        sphereObstacleRadius.push_back(randomObstacleSize());
        if (i==0) {
            Vector3f posInXYPlane = randomPositionInBox();
            posInXYPlane[2] = 0;
            sphereObstacles.push_back(posInXYPlane);
            continue;
        }
        sphereObstacles.push_back(randomPositionInBox());
        
    }
   
}

// evalF applies the three rules of boid flocking behavior
// for a given state, evaluate f(X,t)
vector<Vector3f> SimpleSystem::evalF(vector<Vector3f> state)
{
	vector<Vector3f> f;
	Vector3f flockCenter = centerOfMass(state);

    // load the goal's pos and vel in this state
    Vector3f goalPos = state[0];
    Vector3f goalVel = state[1];

    f.push_back(goalVel);
    f.push_back(evalViscousDrag(goalVel));

	// EVALUATE BIRDS
	for(int i=birdStartIndex; i < predatorStartIndex; i+=2) {
		// get the particles position and velocity
		Vector3f pos_i = state[i];
		Vector3f vel_i = state[i+1];

		Vector3f forces = Vector3f::ZERO;

		// Separation
		Vector3f sepForce = Vector3f::ZERO;
		float numTooClose = 0;
		// Alignment
		Vector3f alignForce = Vector3f::ZERO;
		float numNearby = 0;
		// Cohesion 
		Vector3f cohesiveForce = Vector3f::ZERO;
        // Goal
        Vector3f goalForce = (goalPos - pos_i);
        // Evade
        Vector3f evadeForce = Vector3f::ZERO;

        // For each other bird, determine separation and alignment forces
		for(int j=birdStartIndex; j < predatorStartIndex; j+=2) {
			Vector3f pos_j = state[j];
			Vector3f vel_j = state[j+1];

			if (i == j) {
				continue;
			}

			// Separation
			Vector3f diff = pos_i - pos_j;
			float dist = diff.abs();
			if ( dist < minSeparation ) {
				sepForce+= (diff.normalized() / dist) ;
				numTooClose++;
			}

			// Alignment
			if ( dist < neighborCutoff ) {
				alignForce+= vel_j;
				numNearby++;
			}

		}

        // Separation
		if (numTooClose > 0) { // avoid dividing by zero
			sepForce*= (1/numTooClose); // average the separation forces
		}

        // Alignment
		if (numNearby > 0) { // avoid dividing by zero
			alignForce*= (1/numNearby); //average the align force
			alignForce-= vel_i; // Implement Reynolds: Change = Desired - Current
		}

        // Evade
        float scatter = 1.0f;
        for(int p=predatorStartIndex; p < state.size(); p+=2) {
            Vector3f predator_pos = state[p];
            Vector3f diff = pos_i - predator_pos;
            float dist = diff.abs();
            if ( dist < neighborCutoff ) {
                scatter = 0.5;
                evadeForce+= (2*diff.normalized()/dist);
            }
        }

		// Cohesion
		Vector3f perceived_center = perceivedCenter(flockCenter, pos_i);
		cohesiveForce = (perceived_center - pos_i);
		
		forces+= sepForce * separationWeight;
		forces+= alignForce * alignWeight * scatter;
		forces+= cohesiveForce * cohesiveWeight * scatter;
        forces+= goalForce * goalWeight * scatter;
        forces+= evadeForce;
        forces+= boundPosition(pos_i); // Stay in Bounds
        forces+= avoidObstacles(pos_i); 
        

        // Limit the velocity of the birds
        Vector3f vel_new = vel_i + forces;
        vel_new = limitBirdVelocity(vel_new, getSpeediness(i));
		forces.normalize();
        forces+= evalViscousDrag(vel_new);

		f.push_back(vel_new);        
		f.push_back(forces);
	}

    // EVALUATE PREDATORS
    for(int p=predatorStartIndex; p < state.size(); p+=2) {
        // get the particles position and velocity
        Vector3f pos_pred = state[p];
        Vector3f vel_pred = state[p+1];
        Vector3f forces = Vector3f::ZERO;

        // find the closest prey and follow it
        int birdIndex = findClosestPrey(state, pos_pred);
        forces+= (state[birdIndex] - pos_pred);

        // avoid the other predators
        for(int j=predatorStartIndex; j < state.size(); j+=2) {
            if (p == j) { continue; }

            Vector3f pos_other_pred = state[j];
            Vector3f diff = (pos_pred - pos_other_pred);
            float dist = diff.abs();
            if (dist < neighborCutoff) {
                if (dist == 0.0) {
                    dist = 0.01;
                    diff = Vector3f(1,1,0);
                }
                forces+= (diff.normalized() / dist);
            }
        }

        forces+= boundPosition(pos_pred); // Stay in Bounds
        forces+= avoidObstacles(pos_pred);

        // set velocity and aceleration vectors
        Vector3f vel_new = vel_pred + forces;
        vel_new = limitBirdVelocity(vel_new, getSpeediness(p));
        forces.normalize();
        forces+= evalViscousDrag(vel_new);

        f.push_back(vel_new);
        f.push_back(forces);
    }

	return f;
}

// takes a full state vector and computs the center of mass of just the birds in the state
Vector3f SimpleSystem::centerOfMass(vector<Vector3f> state)
{
	Vector3f summedPos = Vector3f::ZERO;
	for(int i=birdStartIndex; i < predatorStartIndex; i+=2) {
		summedPos+= state[i];
	}
	summedPos*= (1.0 / m_numBirds);
	return summedPos;
}

Vector3f SimpleSystem::perceivedCenter(Vector3f centerOfMass, Vector3f position) 
{
	Vector3f perceived = centerOfMass - (position/m_numParticles);
	perceived*= m_numParticles/ (m_numParticles-1);
	return centerOfMass;
}

int SimpleSystem::getRandomBirdPositionIndex()
{
    return (int)(rand() % m_numParticles + 1) * 2;
}

Vector3f SimpleSystem::getPositionAtIndex(int ind)
{
    return getState()[ind];
}

void SimpleSystem::step() { 
    time_step++; 
    updateGoal(time_step);
}

// render the system (ie draw the particles/birds)
void SimpleSystem::draw()
{
    /* DRAW THE FLOCK CENTER */
	Vector3f flockCenter = centerOfMass(m_vVecState);
	glPushMatrix();
	glTranslatef(flockCenter[0], flockCenter[1], flockCenter[2]);
	glutSolidSphere(0.075f,10.0f,10.0f);
	glPopMatrix();

    /* DRAW THE GOAL */
    glPushMatrix();
    glTranslatef(m_goalPos[0], m_goalPos[1], m_goalPos[2]);
    glEnable(GL_COLOR_MATERIAL);
    glColor3f(.8, .2, .3);
    glDisable(GL_COLOR_MATERIAL);
    glutSolidSphere(0.075f,10.0f,10.0f);
    glPopMatrix();

    /* DRAW THE BIRDS */
	for (int i=birdStartIndex; i < predatorStartIndex; i+=2) {

		Vector3f pos = m_vVecState[i]; // PARTICLE POSITION
		glPushMatrix();
		glTranslatef(pos[0], pos[1], pos[2] );
		glEnable(GL_COLOR_MATERIAL);
		glColor3f(.2 + 0.2*getSpeediness(i), .2, (i/2)*1.0/m_numBirds);
		glDisable(GL_COLOR_MATERIAL);

		if (areParticlesVisible) { // display birds
			Vector3f vel = m_vVecState[i+1].normalized(); // PARTICLE VELOCITY
			float angle = rad_to_deg (acos(Vector3f::dot(vel, Vector3f::FORWARD)));
		    glRotatef(angle, vel.x(), vel.y(), vel.z());
		    drawDove();
		} // otherwise display spheres
		else {
			glutSolidSphere(0.075f,10.0f,10.0f);
		}
		
		glPopMatrix();
	}

    /* DRAW THE PREDATORS */
    for (int j = predatorStartIndex; j < m_vVecState.size(); j+=2) {
        Vector3f predator_pos = m_vVecState[j]; // Predator POSITION
        // cout << "Drawing predator at ";
        // predator_pos.print();
        // cout << endl;
        glPushMatrix();
        glTranslatef(predator_pos[0], predator_pos[1], predator_pos[2] );
        glEnable(GL_COLOR_MATERIAL);
        glColor3f(0.54, 0.27, 0.074);
        glDisable(GL_COLOR_MATERIAL);
        if (areParticlesVisible) { // display birds
            Vector3f vel = m_vVecState[j+1].normalized(); // PARTICLE VELOCITY
            float angle = rad_to_deg (acos(Vector3f::dot(vel, Vector3f::FORWARD)));
            glScalef(1.5, 1.5, 1.5);
            glRotatef(angle, vel.x(), vel.y(), vel.z());
            drawDove();
        } // otherwise display spheres
        else {
            glutSolidSphere(0.1f,10.0f,10.0f);
        }       
        glPopMatrix();
    }

    drawBoundingVertices();
    drawObstacles();
}

void SimpleSystem::drawBoundingVertices() {
    float xDim = boxDims.x();
    float yDim = boxDims.y();
    float zDim = boxDims.z();

    for(int x = -xDim; x <= xDim; x+= (2*xDim)) {
       for(int y = -yDim; y <= yDim; y+= (2*yDim)) {
            for(int z = -zDim; z <= zDim; z+= (2*zDim)) {
                glPushMatrix();
                glTranslatef(x, y, z);
                glEnable(GL_COLOR_MATERIAL);
                glColor3f(.8, .8, .8);
                glDisable(GL_COLOR_MATERIAL);
                glutSolidSphere(0.075f,10.0f,10.0f);
                glPopMatrix();
            }
        } 
    }
}

void SimpleSystem::drawObstacles() {
    for(int i=0; i < sphereObstacles.size(); i++) {
        Vector3f obstacle_pos = sphereObstacles[i];
        float sphere_radius = sphereObstacleRadius[i];

        glPushMatrix();
        glTranslatef(obstacle_pos.x(), obstacle_pos.y(), obstacle_pos.z());
        glEnable(GL_COLOR_MATERIAL);
        glColor3f(1, 0.8549, 0.725);
        glDisable(GL_COLOR_MATERIAL);
        glutSolidSphere(sphere_radius,10.0f,10.0f);
        glPopMatrix();
    }
}


inline void SimpleSystem::drawDove()
{
    //cout << "start" << endl;
    for( unsigned int i=0; i < vecf.size(); i++ )
    {
        glBegin(GL_TRIANGLES);
        glNormal3d(vecn[vecf[i][2]-1][0], vecn[vecf[i][2]-1][1], vecn[vecf[i][2]-1][2]);
        //cout<< "end" << endl;
        glVertex3d(vecv[vecf[i][0]-1][0], vecv[vecf[i][0]-1][1], vecv[vecf[i][0]-1][2]);
        glNormal3d(vecn[vecf[i][5]-1][0], vecn[vecf[i][5]-1][1], vecn[vecf[i][5]-1][2]);
        glVertex3d(vecv[vecf[i][3]-1][0], vecv[vecf[i][3]-1][1], vecv[vecf[i][3]-1][2]);
        glNormal3d(vecn[vecf[i][8]-1][0], vecn[vecf[i][8]-1][1], vecn[vecf[i][8]-1][2]);
        glVertex3d(vecv[vecf[i][6]-1][0], vecv[vecf[i][6]-1][1], vecv[vecf[i][6]-1][2]);
        glEnd();
    }
    //cout<< "end" << endl;
}

void SimpleSystem::loadDove()
{
    std::ifstream infile("dove_simple_test.obj");
    char buffer[2048];
    vecn.clear();
    vecf.clear();

    while( infile.getline(buffer, 2048) )
    {
        stringstream ss(buffer);
        Vector3f v;
        string s;
        ss >> s;

        if ( s == "v" )
        {
            ss >> v[0] >> v[1] >> v[2];
            vecv.push_back(v);
        }
        else if ( s == "vn" )
        {
            ss >> v[0] >> v[1] >> v[2];
            vecn.push_back(v);
        }
        else if ( s == "f" )
        {
            vector<unsigned> vec;
            char delimiter = '/';
            string s;
            for( int i = 0; i < 3; i++)
            {
            ss >> s;
            int i1, i2, i3;
            stringstream face_ind(s);
            face_ind >> i1 >> delimiter >> i2 >> delimiter >> i3;
            vec.push_back(i1);
            vec.push_back(i2);
            vec.push_back(i3);
            }
            vecf.push_back(vec);
        }
    }
}

float SimpleSystem::rad_to_deg(float rad)
{
	return (rad *180) / M_PI;
}

void SimpleSystem::shiftRoot(Vector3f delta) {
    m_goalVel+=delta;
    m_vVecState[1] = m_goalVel;
}

void SimpleSystem::setGoalPattern(int patternId) {
    goalPatternId = patternId;
}

void SimpleSystem::updateGoal(int time_step) {
    switch (goalPatternId) {
        case 1: // Circular
        {
            Vector3f circular = Vector3f(2*cos(time_step/30.0f), 1.5*sin(time_step/30.0f), 0);
            m_vVecState[0] = circular;
            m_vVecState[1] = Vector3f::ZERO;
            break;
        }
        case 2: // Zig-zag
            break;

        case 0: // Static (user-input)
        default:
            break;
    }
    m_goalPos = m_vVecState[0];
    m_goalVel = m_vVecState[1];
}

int SimpleSystem::findClosestPrey(vector<Vector3f> state, Vector3f predator_pos) {
    float smallestDistanceToPrey = 10000;
    int indexOfPrey = birdStartIndex;

    for(int i=birdStartIndex; i < predatorStartIndex; i+=2) {
        Vector3f pos_prey = state[i]; 
        float dist = (predator_pos - pos_prey).abs();
        if ( dist < smallestDistanceToPrey ) {
            smallestDistanceToPrey = dist;
            indexOfPrey = i;
        }
    }
    return indexOfPrey;
}

Vector3f SimpleSystem::limitBirdVelocity(Vector3f vel, float speediness) {
    if (vel.abs() > maxVelocityBird) {
        vel.normalize();
        vel*= (maxVelocityBird + (speediness * speedVariation));
    }
    return vel;
}

Vector3f SimpleSystem::limitPredatorVelocity(Vector3f vel, float speediness) {
    if (vel.abs() > maxVelocityPredator) {
        vel.normalize();
        vel*= (maxVelocityPredator + (speediness * speedVariation));
    }
    return vel;
}

Vector3f SimpleSystem::boundPosition(Vector3f position) {
    Vector3f boundForce = Vector3f::ZERO;
    for(int i=0; i < 3; i++) {
        float distFromEdge = abs(position[i]) - boxDims[i];
        if (distFromEdge > -reboundZone) {
            int sign = (position[i] > 0) ? -1 : 1; // sign of repelling force
            boundForce[i] = (distFromEdge + reboundZone) * sign;

        }  
    }

    return boundForce * reboundFactor; // bounding force
}

Vector3f SimpleSystem::randomPositionInBox() {
    Vector3f randPos = Vector3f::ZERO;
    for(int i=0; i<3; i++) {
        float pos = randf() * boxDims[i] * 2;
        pos-= boxDims[i];
        randPos[i] = pos;
    }
    return randPos;
}

float SimpleSystem::randomObstacleSize() {
    float baseSize = 0.1f;
    float randDiff = randf() * 0.5;
    return baseSize + randDiff;
}

Vector3f SimpleSystem::avoidObstacles(Vector3f position) {
    Vector3f avoidForce = Vector3f::ZERO;
    for(int i=0; i < sphereObstacles.size(); i++) {
        Vector3f pos_obs = sphereObstacles[i];
        float obs_rad = sphereObstacleRadius[i];
        Vector3f diff = position - pos_obs;
        float dist = diff.abs();
        if (dist < obs_rad + obstacleReboundZone) {
            avoidForce+= diff.normalized() / dist;
        }
    }
    return avoidForce;
}

float SimpleSystem::getSpeediness(int bird_index) {
    int ind = (bird_index - birdStartIndex) / 2;
    return speediness[ind];
}

float SimpleSystem::getDaringness(int bird_index) {
    int ind = (bird_index - birdStartIndex) / 2;
    return daringness[ind];
}

float SimpleSystem::getSociableness(int bird_index) {
    int ind = (bird_index - birdStartIndex) / 2;
    return sociableness[ind];
}