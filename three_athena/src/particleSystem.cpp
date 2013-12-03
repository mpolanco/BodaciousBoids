#include "particleSystem.h"
ParticleSystem::ParticleSystem(int nParticles):m_numParticles(nParticles){
	areSpringsVisible = false;
	areParticlesVisible = true;
	wind = false;
	rootDelta = Vector3f::ZERO;
    time_step = 0;
}
