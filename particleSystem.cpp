#include "particleSystem.h"

float rand_uniform(float low, float hi) {
   float abs = hi - low;
   float f = (float)rand() / RAND_MAX;
   f *= abs;
   f += low;
   printf("rand %.2f\n", f);
   return f;
}

ParticleSystem::ParticleSystem(int nParticles):m_numParticles(nParticles){
}
