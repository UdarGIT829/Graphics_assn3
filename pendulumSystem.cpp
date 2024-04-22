#include <iostream>
#include "pendulumSystem.h"

/* 

//declare constants
const Vector3f g(0, -9.8, 0);   // gravity vector (y direction)
const float m = 0.05;           // mass constant
const float kDrag = 0.05;       // viscous drag constant
const float kSpring = 1.5;      // spring constant
const float restingLen = 0.02;  // resting spring length

Spring::Spring() {


}

Spring::Spring(float r) {

    restLen = r;

}

void Spring::addConnection(int i) {

    connections.push_back(i);

}

vector<int> Spring::getConnections() {

    return connections;

}


PendulumSystem::PendulumSystem(int numParticles):ParticleSystem(numParticles) {

	printf("INITIALIZING PENDULUM \n");

	m_numParticles = numParticles;

	vector<Vector3f> initialConds;  //initial position and velocity

	// fill in code for initializing the state based on the number of particles

	for (int i = 0; i < m_numParticles; i++) {

        float f = rand_uniform(-0.5f, 0.5f);

		Vector3f X, V;

        X = Vector3f(0.5 + f, 1.0 + f, f);
        V = Vector3f(0.0, 0.0, 0.0);

        initialConds.push_back(X);
        initialConds.push_back(V);

        
        if (i == 0) {   // fixed initial particle in spring
            springs.push_back(Spring());
        } 
    
        else {         //other particles
            Spring spring = Spring();
       
            spring.addConnection(i - 1);       //connect neighbor springs
            springs[i - 1].addConnection(i);
            springs.push_back(spring);
        }

		m_vVecState.push_back(X);	//init simple system 
		m_vVecState.push_back(V);

    }
}



vector<Vector3f> getPositions(vector<Vector3f> &state) {

    vector<Vector3f> positions;

	//we want to collect the X here

    for (unsigned int i = 0; i < state.size(); i += 2) {
        positions.push_back(state[i]);
    }
    return positions;
}



vector<Vector3f> getVelocities(vector<Vector3f> &state) {

    vector<Vector3f> velocities;

	//we want to collect the V here

    for (unsigned int i = 1; i < state.size(); i += 2) {
        velocities.push_back(state[i]);
    }
    return velocities;
}

// TODO: implement evalF

// for a given state, evaluate f(X,t)

vector<Vector3f> PendulumSystem::evalF(vector<Vector3f> state)

{

	vector<Vector3f> f;

	// // YOUR CODE HERE

    vector<Vector3f> positions = getPositions(state);
    vector<Vector3f> velocities = getVelocities(state);

    f.push_back(Vector3f(0.0, 0.0, 0.0));       // fix initial vertex location of pendulum
    f.push_back(Vector3f(0.0, 0.0, 0.0));

    // TODO 4.1: implement evalF

    for (unsigned int i = 1; i < positions.size(); i++) {

        Vector3f springForces(0, 0, 0); //spring forces
        const Vector3f gravity = m * g; //gravity force (mass * gravity)
        const Vector3f viscousDrag = -kDrag * velocities[i];    //viscous drag force (drag const * velocity vector)
        

        const vector<int> connections = springs[i].getConnections();    //load connections of spring

        for (long unsigned int j = 0; j < connections.size(); j++) {

			int c = connections[j];

            const Vector3f d = positions[i] - positions[c]; //get distance
            const Vector3f F = -kSpring * (d.abs() - restingLen) * d.normalized();  //distance - resting length * dist normalized

            springForces += F;

        }

        const Vector3f netForce = gravity + viscousDrag + springForces;   // net force

        const Vector3f acceleration = netForce / m; // X'' = F(X, X')

        // dX/dt = f(X, t) = <v, F(x,v)>
        f.push_back(velocities[i]);
        f.push_back(acceleration);
    }
    return f;
}



// render the system (ie draw the particles)

void PendulumSystem::draw() {

	for (int i = 0; i < m_numParticles; i++) {

		Vector3f pos = getState()[i];//  position of particle i. YOUR CODE HERE

		glPushMatrix();
		glTranslatef(pos[0], pos[1], pos[2] );
		glutSolidSphere(0.075f,10.0f,10.0f);
		glPopMatrix();
	}

}

*/

PendulumSystem::PendulumSystem(int numParticles):ParticleSystem(numParticles)   //constructor
{
	m_numParticles = numParticles;

	this->mass = 0.1f;                  //vertex mass
	this->k4f = 0.01f;                  //viscous drag
	this->k4Spring = 3.0f;              //drag for entire spring
	this->init_spring_length = 0.2f;    //initial spring length

	m_vVecState.push_back(Vector3f(0, 0, 0));     //init default vertex at origin (0,0,0)
	m_vVecState.push_back(Vector3f(0, 0, 0));

	for (int i = 1; i < m_numParticles; i++) {  //loop through all particles, note track position and velocity of each vertex
	
		m_vVecState.push_back(Vector3f(i * init_spring_length, 0, 0)); 
		m_vVecState.push_back(Vector3f(0, 0, 0));   // point velocity (0, 0, 0)

		m_springs.push_back(Spring(i, i - 1, init_spring_length, k4Spring));       //create spring between vertex i and vertex i - 1
	}
}


// TODO: implement evalF

// for a given state, evaluate f(X,t)

vector<Vector3f> PendulumSystem::evalF(vector<Vector3f> state)
{
	// half of f store the velocity of states
	// the other store the a=F(x,v)

	vector<Vector3f> f; //stores velocity and force given a state

	for (int i = 0; i < 2 * m_numParticles; i++) {  //every other vertex
		
		if (i % 2 == 0) {   //push velocity here, store velocity first then force values after
            f.push_back(state[i + 1]); 
            continue;
        }     

		if (i == 1) {   // initial vertex will have no force, should be static in place
			f.push_back(Vector3f::ZERO);    //constant in vecmath library for zero
			continue;
		}

		//note: total force = gravity + viscous drag + spring_force	

		Vector3f totalForces = (-mass) * Vector3f(0, 9.8f, 0);    //add gravity vector
        
		totalForces = ((-k4f)* state[i]) + totalForces;           //add viscous drag vector, negative for slow

		totalForces = totalForces + getSpringForce(i/2, state);   //add spring force values
		
		totalForces = (1 / mass) * totalForces;                   //divide by mass. note (X,t) = (1/mass) * F
		
        f.push_back(totalForces);
	}
	return f;
}

//render the system (ie draw the particles)
void PendulumSystem::draw()
{
	for (int i = 0; i < m_numParticles; i++) {

		Vector3f pos = m_vVecState[i*2] ;   //position of particle i. YOUR CODE HERE

		glPushMatrix();
		glTranslatef(pos[0], pos[1], pos[2] );
		glutSolidSphere(0.075f,10.0f,10.0f);
		glPopMatrix();
	}
	drawSprings();
}

void PendulumSystem::addSpring(Spring spring){  //add spring to vector of all springs
	m_springs.push_back(spring);
}

Vector3f PendulumSystem::getSpringForce(int i, vector<Vector3f> state){ //iterate and get net sum of forces

	Vector3f spring_force = Vector3f::ZERO; //init to zero forces for 1st vertex and calls for successive vertices

	for (long unsigned int j = 0; j < m_springs.size(); j++) {  //loop through all springs

        //for both left/right vertices,  calculate spring force distance
		if (m_springs[j].leftnode == i) {
			Vector3f d = getPosition(i, state) - getPosition(m_springs[j].rightnode, state);
			spring_force = spring_force + (-m_springs[j].k_spring) * (d.abs() - m_springs[j].static_length) * (d.normalized());
		}

		else if (m_springs[j].rightnode == i) {
			Vector3f d = getPosition(i, state) - getPosition(m_springs[j].leftnode, state);
			spring_force = spring_force + ((-m_springs[j].k_spring) * (d.abs() - m_springs[j].static_length)) * (d.normalized());
		}
	}
	return spring_force;
}

Vector3f PendulumSystem::getvelocity(int i, vector<Vector3f> state){
	return state[2*i+1];
}

Vector3f PendulumSystem::getPosition(int i, vector<Vector3f> state){
	return state[i*2];
}

void PendulumSystem::drawSprings(){
	glLineWidth(5);
	glBegin(GL_LINES);
	
	for (long unsigned int i = 0;i < m_springs.size(); i++)	{
		glVertex3f(getPosition(m_springs[i].leftnode,getState()).x(), getPosition(m_springs[i].leftnode, getState()).y(), getPosition(m_springs[i].leftnode, getState()).z());
		glVertex3f(getPosition(m_springs[i].rightnode, getState()).x(), getPosition(m_springs[i].rightnode, getState()).y(), getPosition(m_springs[i].rightnode, getState()).z());
	}

	glEnd();
}
