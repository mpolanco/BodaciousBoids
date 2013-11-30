
#include "simpleSystem.h"

using namespace std;

SimpleSystem::SimpleSystem()
{
	m_numParticles = 1;
	m_vVecState.push_back(Vector3f(0, -1.0, 0));
	loadDove();
}

// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> SimpleSystem::evalF(vector<Vector3f> state)
{
	vector<Vector3f> f;

	// YOUR CODE HERE
	for (int i = 0; i < state.size(); ++i)
	{
		/* code */
		f.push_back(Vector3f(-1.0 * state[i].y(), state[i].x(), 0.0));
	}
	return f;
}

// render the system (ie draw the particles)
void SimpleSystem::draw()
{
	Vector3f pos = m_vVecState[0];//YOUR PARTICLE POSITION
	if (areParticlesVisible)
	{
		glPushMatrix();
		glTranslatef(pos[0], pos[1], pos[2] );
		//glutSolidSphere(0.075f,10.0f,10.0f);
		drawDove();

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