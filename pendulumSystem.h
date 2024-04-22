#ifndef PENDULUMSYSTEM_H
#define PENDULUMSYSTEM_H

#include <vecmath.h>
#include <vector>
#include <GL/glut.h>
#include "particleSystem.h"

struct Spring       //string struct
{
	int leftnode;           //left vertex of triangle
	int rightnode;          //right vertex of triangle
	float static_length;    //length of spring
	float k_spring;         //rk4 for spring

	Spring(int leftnode, int rightnode, float static_length, float k_spring) {

		this->leftnode = leftnode;          
		this->rightnode = rightnode;
		this->static_length = static_length;
		this->k_spring = k_spring;
	}
};

class PendulumSystem: public ParticleSystem
{
public:

	PendulumSystem(int numParticles);
	
	vector<Vector3f> evalF(vector<Vector3f> state);
	
	void draw();

	void addSpring(Spring spring);  //add spring function
    void drawSprings();	// draw springs
	
	Vector3f getSpringForce(int i, vector<Vector3f> state); //sum of springs' force at a given vertex.
	Vector3f getvelocity(int i, vector<Vector3f> state);    //velocity of a point
	Vector3f getPosition(int i, vector<Vector3f> state);    //position of a point

private:
	
	float mass;                 // mass of vertex
	float k4f;                  // k4 for viscous drag
	float k4Spring;             // k4 for all springs
	float init_spring_length;    // static initial length of spring

	vector<Spring> m_springs;   // vector to track all springs
};



#endif