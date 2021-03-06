#include "ClothSystem.h"
#include <iostream>


//TODO: Initialize here
ClothSystem::ClothSystem()
{
    loadDove();
	m_numParticles = CLOTH_WIDTH * CLOTH_HEIGHT;
	for (int i = 0; i < CLOTH_HEIGHT; ++i)
	{
		for (int k = 0; k < CLOTH_WIDTH; ++k)
		{
			//particle position
			m_vVecState.push_back(Vector3f(CLOTH_DETAIL * k, 0, CLOTH_DETAIL * i));
			//partivel velocity
			m_vVecState.push_back(Vector3f(0.0,0.0,0.0));
			//add spring
			std::vector<Spring> v;
			springTies.push_back(v);
		}
	}
	initStructuralSprings();
	initShearSprings();
	initFlexSprings();
	initMasses();
}

void ClothSystem::setSpring(int start, int end, float springConst, float restLength)
{
	std::vector<Spring> vec = springTies[start/2];
	vec.push_back(Spring(start, end, springConst, restLength));
	springTies[start/2] = vec;
}

inline int ClothSystem::indexOf(int row, int col)
{
	return (col + row * CLOTH_WIDTH) * 2;
}

void ClothSystem::initMasses()
{
	for (int i = 0; i < m_numParticles; ++i)
	{
		masses.push_back( 1.0 );
	}
}

void ClothSystem::initStructuralSprings()
{
	float springConst = 200.0;
	float restLength = 1.0 * CLOTH_DETAIL;

	for (int i = 0; i < CLOTH_HEIGHT; ++i)
	{
		for (int k = 0; k < CLOTH_WIDTH; ++k)
		{
			int ind = indexOf(i, k);
            int up = indexOf(i-1, k);
            int down = indexOf(i+1, k);
            int left = indexOf(i, k-1);
            int right = indexOf(i, k+1);

            if (i != 0)
            {
            	setSpring(ind, up, springConst, restLength);
            }
            if (i != CLOTH_HEIGHT - 1)
            {
            	setSpring(ind, down, springConst, restLength);
            }
            if (k != 0)
            {
            	setSpring(ind, left, springConst, restLength);
            }
            if (k != CLOTH_WIDTH - 1)
            {
            	setSpring(ind, right, springConst, restLength);
            }	
		}
	}

}

void ClothSystem::initShearSprings()
{
	float springConst = 200.0;
	float restLength = sqrt(2) * CLOTH_DETAIL;

	for (int i = 0; i < CLOTH_HEIGHT; ++i)
	{
		for (int k = 0; k < CLOTH_WIDTH; ++k)
		{
			int ind = indexOf(i, k);
            int upLeft = indexOf(i-1, k-1);
            int downRight = indexOf(i+1, k+1);
            int downLeft = indexOf(i+1, k-1);
            int upRight = indexOf(i-1, k+1);

            if ( i != 0 && k != CLOTH_WIDTH - 1 )
            {
            	setSpring(ind, upRight, springConst, restLength);
            }
            if ( i != 0 && k != 0)
            {
            	setSpring(ind, upLeft, springConst, restLength);
            }
            if ( i != CLOTH_HEIGHT - 1 && k != 0 )
            {
            	setSpring(ind, downLeft, springConst, restLength);
            }
            if (i != CLOTH_HEIGHT - 1 && k != CLOTH_WIDTH - 1)
            {
            	setSpring(ind, downRight, springConst, restLength);
            }	
		}
	}
}
void ClothSystem::initFlexSprings()
{
	float springConst = 200.0;
	float restLength = 2.0 * CLOTH_DETAIL;

	for (int i = 0; i < CLOTH_HEIGHT; ++i)
	{
		for (int k = 0; k < CLOTH_WIDTH; ++k)
		{
			int ind = indexOf(i, k);
            int up = indexOf(i-2, k);
            int down = indexOf(i+2, k);
            int left = indexOf(i, k-2);
            int right = indexOf(i, k+2);

            if (i > 1)
            {
            	setSpring(ind, up, springConst, restLength);
            }
            if (i < 6)
            {
            	setSpring(ind, down, springConst, restLength);
            }
            if (k > 1)
            {
            	setSpring(ind, left, springConst, restLength);
            }
            if (k < 6)
            {
            	setSpring(ind, right, springConst, restLength);
            }	
		}
	}
}

// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> ClothSystem::evalF(vector<Vector3f> state)
{
	vector<Vector3f> f;
	// YOUR CODE HERE
	for (int i = 0; i < state.size(); i += 2)
	{
		//get particle mass 
		float mass = masses[i/2];

		//get position and velocity of particle
		Vector3f particlePos = state[i];
		Vector3f particleVel = state[i+1];
		Vector3f particleForce = Vector3f(0.0, 0.0, 0.0);
		
		//calculate forces
		//gravitational force
		particleForce += mass * GRAVITY;

		//drag
		particleForce += mass * -1.0 * DRAG * particleVel;

		//wind
		if (wind)
		{
			particleForce += windForce * (rand() % 2 + 0);
		}

		//spring force
		std::vector<Spring> particleSprings = springTies[i/2];
		for (int k = 0; k < particleSprings.size(); ++k)
		{
			Spring particleSpring = particleSprings[k];

			Vector3f pos1 = state[particleSpring.particleInd1];
			Vector3f pos2 = state[particleSpring.particleInd2];

			Vector3f dist = pos1 - pos2;

			Vector3f spForce = -1.0 * particleSpring.k * (dist.abs() - particleSpring.restLength) * (dist/dist.abs());

			particleForce += mass * spForce;
		}

		//push dx (velocity)
		f.push_back(particleVel);
		//push dv (acceleration)
		f.push_back(particleForce/mass);
	}

	//keep initial particles at rest
	f[0] = rootVel;
	f[1] = Vector3f(0,0,0);
	f[(CLOTH_WIDTH-1) * 2] = rootVel;
	f[(CLOTH_WIDTH-1) * 2 + 1] = Vector3f(0,0,0);

	shiftRoot(Vector3f::ZERO);

	return f;
}

///TODO: render the system (ie draw the particles)
void ClothSystem::draw()
{
	for (int i = 0; i < m_numParticles; i++) {
		Vector3f pos = m_vVecState[i*2];
		if (areParticlesVisible)
		{
			glPushMatrix();
			glTranslatef(pos[0], pos[1], pos[2] );
            drawDove();
			
			glPopMatrix();
		}

		if (areSpringsVisible)
		{
			std::vector<Spring> springs = springTies[i];
			for (int k = 0; k < springs.size(); ++k)
			{
				glBegin(GL_LINES);
				glVertex(pos);
				glVertex(m_vVecState[springs[k].particleInd2]);
				glEnd();
			}
		}
	}
}

inline void ClothSystem::drawDove()
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

void ClothSystem::loadDove()
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

