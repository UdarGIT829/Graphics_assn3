#include "ClothSystem.h"

//TODO: Initialize here
ClothSystem::ClothSystem()
{
}


// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> ClothSystem::evalF(vector<Vector3f> state)
{
	vector<Vector3f> f;

    for (int i = 0; i < (int)state.size(); ++i) {
        f.push_back(Vector3f(-state[i].y(), state[i].x(), 0));
    }
    return f;
	
}

// TODO: render the system (ie draw the particles)
void ClothSystem::draw()
{
}

