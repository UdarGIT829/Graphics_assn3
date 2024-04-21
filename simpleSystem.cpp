#include "simpleSystem.h"

using namespace std;

SimpleSystem::SimpleSystem()	//constructor
{
	m_vVecState.push_back(Vector3f(1.0, 0.0, 0.0));	//init simple system 
	m_numParticles = 1;	//default 1 particle
}

// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> SimpleSystem::evalF(vector<Vector3f> state)
{
	vector<Vector3f> f;

	// YOUR CODE HERE
	for (int i = 0; i < (int)state.size(); i++){		//for given state, get derivative f(X,t)

		//note: if X of t = (x y z) coordinates, then f(X,t) = (-y x 0)
		f.push_back(Vector3f(-state[i].y(), state[i].x(), 0));
	}

	return f;
}

// render the system (ie draw the particles)
void SimpleSystem::draw()
{
	Vector3f pos = getState()[0];	//YOUR PARTICLE POSITION
	    
	glPushMatrix();
	glTranslatef(pos[0], pos[1], pos[2] );
	glutSolidSphere(0.075f,10.0f,10.0f);
	glPopMatrix();
}