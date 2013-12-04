
#include "simpleSystem.h"

using namespace std;

// Repurposed for Flock

SimpleSystem::SimpleSystem(int numBirds, int numPredators)
{
    GOAL_DEFAULT = 0;
    GOAL_CIRCULAR = 1;
    GOAL_ZIGZAG = 2;

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
	}

    predatorStartIndex = m_vVecState.size();
    for (int j=0; j < numPredators; j++) {
        Vector3f pos = Vector3f(1, 1, 1);
        Vector3f vel = Vector3f(0, 0, 0);

        m_vVecState.push_back(pos);
        m_vVecState.push_back(vel);
    }

	minSeparation = 1.0f;
	neighborCutoff = 3.0f;
    maxVelocityBird = 1.2f;
    maxVelocityPredator = 0.8f * maxVelocityBird;
    predatorSeparation = 2.5f;
	cout << "Init: articles should have minimum Separation: " << minSeparation << endl;
	MAX_BUFFER_SIZE = 100;
	loadDove();
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

	// for each particle in the state
	// evaluate actual forces, except anchor the first particle
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
			alignForce*= (1.0/8.0f);
		}

		// Cohesion
		Vector3f perceived_center = perceivedCenter(flockCenter, pos_i);
		cohesiveForce = (perceived_center - pos_i)/4; // weight the force so it's not overpowering
		
		forces+= sepForce;
		forces+= alignForce;
		forces+= cohesiveForce;
        forces+= goalForce;	

        // Limit the velocity of the birds
        Vector3f vel_new = vel_i + forces;
        if (vel_new.abs() > maxVelocityBird) {
            vel_new.normalize();
            vel_new*= maxVelocityBird;
        }
		forces.normalize();

		f.push_back(vel_i+forces);
        forces+= evalViscousDrag(vel_i+forces);
		f.push_back(forces);
	}

    for(int p=predatorStartIndex; p < state.size(); p+=2) {
        // get the particles position and velocity
        Vector3f pos_pred = state[p];
        Vector3f vel_pred = state[p+1];
        f.push_back(vel_pred);
        f.push_back(Vector3f::ZERO);
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
		glColor3f(.2, .2, (i/2)*1.0/m_numBirds);
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
        glPushMatrix();
        glTranslatef(predator_pos[0], predator_pos[1], predator_pos[2] );
        glEnable(GL_COLOR_MATERIAL);
        glColor3f(0.54, 0.27, 0.074);
        glDisable(GL_COLOR_MATERIAL);
        glutSolidSphere(0.075f,10.0f,10.0f);        
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
    switch(patternId) {
        case 1:
            break;
        default:
            break;
    }
}

void SimpleSystem::updateGoal(int time_step) {
    m_goalPos = m_vVecState[0];
    m_goalVel = m_vVecState[1];

    switch (goalPatternId) {
        case 1: // Circular
        {
            Vector3f circular = Vector3f(2*cos(time_step/30.0f), 1.5*sin(time_step/30.0f), 0);
            m_goalPos = circular;
            m_vVecState[0] = m_goalPos;
            m_vVecState[1] = Vector3f::ZERO;
            break;
        }
        case 2: // Zig-zag
            break;
        case 0: // Static (user-input)
        default:
            break;
    }
}