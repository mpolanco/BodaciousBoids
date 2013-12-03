
#include "simpleSystem.h"

using namespace std;

// Repurposed for Flock
SimpleSystem::SimpleSystem()
{
	areParticlesVisible = false;
	m_numParticles = 10;

    // push the goal's pos and vel first before any birds
    m_goalPos = Vector3f(0,0,0);
    m_goalVel = Vector3f(0,0,0);
    m_vVecState.push_back(m_goalPos);
    m_vVecState.push_back(m_goalVel);

	for (int i=0; i < m_numParticles; i++) {
		Vector3f pos = Vector3f(i*0.4, randf(), randf());
		Vector3f vel = Vector3f(0, randf(), 0);

		m_vVecState.push_back(pos);
		m_vVecState.push_back(vel);
	}

	minSeparation = 1.0f;
	neighborCutoff = 3.0f;
    maxVelocity = 1.2f;
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
    f.push_back(Vector3f::ZERO);

	// for each particle in the state
	// evaluate actual forces, except anchor the first particle
	for(int i=2; i < state.size(); i+=2) {
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
		for(int j=2; j < state.size(); j+=2) {
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
        if (vel_new.abs() > maxVelocity) {
            vel_new.normalize();
            vel_new*= maxVelocity;
        }
		forces.normalize();

		f.push_back(vel_i+forces);
		f.push_back(forces);
	}

	return f;
}

Vector3f SimpleSystem::centerOfMass(vector<Vector3f> state)
{
	Vector3f summedPos = Vector3f::ZERO;
	float numPos = state.size()/2;
	for(int i=0; i < state.size(); i+=2) {
		summedPos+= state[i];
	}
	summedPos*= (1.0 / numPos);
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
    // Vector3f circular = Vector3f(cos(time_step/90.0f), sin(time_step/90.0f), 0);
    m_goalPos = m_vVecState[0];
    m_goalVel = m_vVecState[1];
}

// render the system (ie draw the particles/birds)
void SimpleSystem::draw()
{
	Vector3f flockCenter = centerOfMass(m_vVecState);
	glPushMatrix();
	glTranslatef(flockCenter[0], flockCenter[1], flockCenter[2]);
	glutSolidSphere(0.075f,10.0f,10.0f);
	glPopMatrix();

    glPushMatrix();
    glTranslatef(m_goalPos[0], m_goalPos[1], m_goalPos[2]);
    glEnable(GL_COLOR_MATERIAL);
    glColor3f(.8, .2, .3);
    glDisable(GL_COLOR_MATERIAL);
    glutSolidSphere(0.075f,10.0f,10.0f);
    glPopMatrix();

	for (int i=0; i < m_numParticles; i++) {

		Vector3f pos = m_vVecState[2*i+2]; // PARTICLE POSITION
		glPushMatrix();
		glTranslatef(pos[0], pos[1], pos[2] );
		glEnable(GL_COLOR_MATERIAL);
		glColor3f(.2, .2, i*1.0/m_numParticles);
		glDisable(GL_COLOR_MATERIAL);

		if (areParticlesVisible) { // display birds
			Vector3f vel = m_vVecState[2*i+1+2].normalized(); // PARTICLE VELOCITY
			float angle = rad_to_deg (acos(Vector3f::dot(vel, Vector3f::FORWARD)));
		    glRotatef(angle, vel.x(), vel.y(), vel.z());
		    drawDove();
		} // otherwise display spheres
		else {
			glutSolidSphere(0.075f,10.0f,10.0f);
		}
		
		glPopMatrix();
	}
}

inline void SimpleSystem::drawDove()
{
    for( unsigned int i=0; i < vecf.size(); i++ )
    {
        glBegin(GL_TRIANGLES);
        glNormal3d(vecn[vecf[i][2]-1][0], vecn[vecf[i][2]-1][1], vecn[vecf[i][2]-1][2]);
        glVertex3d(vecv[vecf[i][0]-1][0], vecv[vecf[i][0]-1][1], vecv[vecf[i][0]-1][2]);
        glNormal3d(vecn[vecf[i][5]-1][0], vecn[vecf[i][5]-1][1], vecn[vecf[i][5]-1][2]);
        glVertex3d(vecv[vecf[i][3]-1][0], vecv[vecf[i][3]-1][1], vecv[vecf[i][3]-1][2]);
        glNormal3d(vecn[vecf[i][8]-1][0], vecn[vecf[i][8]-1][1], vecn[vecf[i][8]-1][2]);
        glVertex3d(vecv[vecf[i][6]-1][0], vecv[vecf[i][6]-1][1], vecv[vecf[i][6]-1][2]);
        glEnd();
    }
}

void SimpleSystem::loadDove()
{
    std::ifstream infile("dovetest.obj");
    char buffer[1024];

    while( infile.getline(buffer, 1024) )
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
    cout << "Goal position: ";
    m_goalPos.print();
    cout << endl;
    m_goalVel+=delta;
    m_vVecState[1] = m_goalVel;
    cout << "New goal velocity: ";
    m_goalVel.print();
    cout << endl;
}
