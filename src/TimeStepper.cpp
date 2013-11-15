#include "TimeStepper.h"

///TODO: implement Explicit Euler time integrator here
void ForwardEuler::takeStep(ParticleSystem* particleSystem, float stepSize)
{
	std::vector< Vector3f > state = particleSystem->getState();
	std::vector< Vector3f > derivative = particleSystem->evalF(state);
	std::vector< Vector3f > newState;

	for (int i = 0; i < state.size(); ++i)
	{
		Vector3f update = state[i] + stepSize * derivative[i];
		newState.push_back(update);
	}
	particleSystem->setState(newState);
}

///TODO: implement Trapzoidal rule here
void Trapzoidal::takeStep(ParticleSystem* particleSystem, float stepSize)
{
	std::vector< Vector3f > state = particleSystem->getState();
	std::vector< Vector3f > derivative = particleSystem->evalF(state);
	std::vector< Vector3f > tempState;
	std::vector< Vector3f > newState;	

	for (int i = 0; i < state.size(); ++i)
	{
		Vector3f update = state[i] + stepSize * derivative[i];
		tempState.push_back(update);
	}

	std::vector< Vector3f > derivative1 = particleSystem->evalF(tempState);
	for (int i = 0; i < state.size(); ++i)
	{
		Vector3f update = state[i] + (stepSize/2.0) * (derivative[i] + derivative1[i]);
		newState.push_back(update);
	}

	particleSystem->setState(newState);
}
