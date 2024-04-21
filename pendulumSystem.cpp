#include <iostream>
#include "pendulumSystem.h"

// Gravity vector (y direction)
const Vector3f g(0, -9.8, 0);
// Mass of an object
const float m = 0.05;
// Viscous drag constant
const float kDrag = 0.05;
// Spring constant
const float kSpring = 1.5;
// Resting spring length
const float restingLen = 0.02;

Spring::Spring() {
    // Any extra logic here.
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

PendulumSystem::PendulumSystem(int numParticles):ParticleSystem(numParticles)
{
	printf("INITIALIZING PENDULUM \n");
	m_numParticles = numParticles;
	vector<Vector3f> initialConds;
	// fill in code for initializing the state based on the number of particles
	for (int i = 0; i < m_numParticles; i++) 
	{
        float f = rand_uniform(-0.5f, 0.5f);
		Vector3f X, V;
        X = Vector3f(0.5 + f, 1.0 + f, f);
        V = Vector3f(0.0, 0.0, 0.0);
        initialConds.push_back(X);
        initialConds.push_back(V);
        // Initial point in the spring, fixed.
        if (i == 0) {
            springs.push_back(Spring());
        } else { // For other particles.
            Spring spring = Spring();
            // Connect both neighbors.
            spring.addConnection(i - 1);
            springs[i - 1].addConnection(i);
            springs.push_back(spring);
        }
		m_vVecState.push_back(X);	//init simple system 
		m_vVecState.push_back(V);
    }
}

vector<Vector3f> getPositions(vector<Vector3f> &state) {
    vector<Vector3f> positions;
	//We want to collect the X here
    for (unsigned int i = 0; i < state.size(); i += 2) {
        positions.push_back(state[i]);
    }
    return positions;
}

vector<Vector3f> getVelocities(vector<Vector3f> &state) {
    vector<Vector3f> velocities;
	//We want to collect the V here
    for (unsigned int i = 1; i < state.size(); i += 2) {
        velocities.push_back(state[i]);
    }
    return velocities;
}
// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> PendulumSystem::evalF(vector<Vector3f> state)
{
	// vector<Vector3f> f;

	// // YOUR CODE HERE

	// return f;
	vector<Vector3f> positions = getPositions(state);
    vector<Vector3f> velocities = getVelocities(state);
    // assert(positions.size() == velocities.size());
    std::vector<Vector3f> f;
    // Fix the initial point.
    f.push_back(Vector3f(0.0, 0.0, 0.0));
    f.push_back(Vector3f(0.0, 0.0, 0.0));
    // TODO 4.1: implement evalF
    for (unsigned int i = 1; i < positions.size(); ++i) {
        //  - gravity
        const Vector3f gravity = m * g;
        //  - viscous drag
        const Vector3f viscousDrag = -kDrag * velocities[i];
        //  - springs
        Vector3f springForces(0, 0, 0);
        const vector<int> connections = springs[i].getConnections();
        for (long unsigned int j = 0; j < connections.size(); ++j) {
			int c = connections[j];
            const Vector3f d = positions[i] - positions[c];
            const Vector3f F = -kSpring * (d.abs() - restingLen) * d.normalized();
            springForces += F;
        }
        // net force
        const Vector3f netForce = gravity + viscousDrag + springForces;
        // X'' = F(X, X')
        const Vector3f acceleration = netForce / m;
        // dX/dt = f(X, t) = <v, F(x,v)>
        f.push_back(velocities[i]);
        f.push_back(acceleration);
    }
    return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw()
{
	for (int i = 0; i < m_numParticles; i++) {

		Vector3f pos = getState()[i];//  position of particle i. YOUR CODE HERE
		glPushMatrix();
		glTranslatef(pos[0], pos[1], pos[2] );
		glutSolidSphere(0.075f,10.0f,10.0f);
		glPopMatrix();
	}
}
