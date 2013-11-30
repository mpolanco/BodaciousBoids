#include "particleSystem.h"
ParticleSystem::ParticleSystem(int nParticles):m_numParticles(nParticles){
	areSpringsVisible = false;
	areParticlesVisible = true;
	wind = false;
	rootVel = Vector3f::ZERO;
}
