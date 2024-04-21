#include "TimeStepper.hpp"

///TODO: implement Explicit Euler time integrator here
void ForwardEuler::takeStep(ParticleSystem* particleSystem, float stepSize)
{
	//given state X, we examine f(X,t) at X, then step to the new state value
	//X(t + h) = X + h * f (X, t) where h is the step size

	vector<Vector3f> X = particleSystem->getState();	//given state X
	vector<Vector3f> F = particleSystem->evalF(X);		//get F

	vector<Vector3f> X_new;	//new X state after stepping

	for (int i = 0; i < (int)X.size(); i++){
		X_new.push_back(X[i] + stepSize * F[i]);	//X + h * f(X,t)
	}

	particleSystem->setState(X_new);	//set new state after calculating
}

///TODO: implement Trapzoidal rule here
void Trapzoidal::takeStep(ParticleSystem* particleSystem, float stepSize)
{
	//average of the force f0 at the current state and the force f1 after an Euler step of stepsize h
	//f0 = f (X, t)
	//f1 = f (X + h * f0, t + h)
	//X(t + h) = X + h/2 * (f0 + f1)

	vector<Vector3f> X = particleSystem->getState();	//given state X
	vector<Vector3f> F0 = particleSystem->evalF(X);		//get F

	vector<Vector3f> X_sum;	//vector for storing X + hf0

	for (int i = 0; i < (int)X.size(); i++){
		X_sum.push_back(X[i] + stepSize * F0[i]);	//X + h * f0
	}

	vector<Vector3f> F1 = particleSystem->evalF(X_sum);		//get f1 = f (X + hf0, t + h)
	vector<Vector3f> X_new;	//new X state after stepping

	for (int i = 0; i < (int)X.size(); i++){
		X_new.push_back(X[i] + ((stepSize/2) * (F0[i] + F1[i])));	//X(t + h) = X + h/2 * (f0 + f1)
	}

	particleSystem->setState(X_new);
} 

// RK4 is given to you
void RK4::takeStep(ParticleSystem* particleSystem, float stepSize)
{	
	vector<Vector3f> X1 = particleSystem->getState();	
	vector<Vector3f> f1 = particleSystem->evalF(X1);
	if (f1.size() >= 1) {		
		vector<Vector3f> X2;
		for (unsigned int i = 0; i < 2 * particleSystem->m_numParticles; i++)
		{
			X2.push_back(X1[i] + (stepSize * f1[i] / 2.0));
		}		
		vector<Vector3f> f2 = particleSystem->evalF(X2);		
		vector<Vector3f> X3;
		for (unsigned int i = 0; i < 2 * particleSystem->m_numParticles; i++)
		{
			X3.push_back(X1[i] + (stepSize * f2[i] / 2.0));
		}		
		vector<Vector3f> f3 = particleSystem->evalF(X3);		
		vector<Vector3f> X4;
		for (unsigned int i = 0; i < 2 * particleSystem->m_numParticles; i++)
		{
			X4.push_back(X1[i] + (stepSize * f3[i]));
		}		
		vector<Vector3f> f4 = particleSystem->evalF(X4);		
		for (unsigned int i = 0; i < 2 * particleSystem->m_numParticles; i++)
		{
			X1[i] = X1[i] + stepSize * ((f1[i] + f2[i] * 2 + f3[i] * 2 + f4[i]) / 6);
		}
	}
	particleSystem->setState(X1);
}