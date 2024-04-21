#ifndef PENDULUMSYSTEM_H
#define PENDULUMSYSTEM_H

#include <vecmath.h>
#include <vector>
#include <GL/glut.h>

#include "particleSystem.h"

// float rand_uniform(float low, float hi) {
//    float abs = hi - low;
//    float f = (float)rand() / RAND_MAX;
//    f *= abs;
//    f += low;
//    printf("rand %.2f\n", f);
//    return f;
// }

class Spring {
public:
    Spring();
    Spring(float r);
    void addConnection(int i);
    std::vector<int> getConnections();
private:
    std::vector<int> connections;
    float restLen;
};

class PendulumSystem: public ParticleSystem
{
public:
	PendulumSystem(int numParticles);
	
	vector<Vector3f> evalF(vector<Vector3f> state);
	
	void draw();
	std::vector<Spring> springs;
	
};

#endif
