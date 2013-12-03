#include "particleSystem.h"
ParticleSystem::ParticleSystem(int nParticles):m_numParticles(nParticles){
	areSpringsVisible = false;
	areParticlesVisible = true;
	wind = false;
	rootDelta = Vector3f::ZERO;
    time_step = 0;
    DRAG_CONSTANT = 0.5;
}

// evaluates the viscous drag force that acts of a particle with velocity vel
Vector3f ParticleSystem::evalViscousDrag(Vector3f vel)
{
    Vector3f D = (-1.0 * DRAG_CONSTANT) * vel;
    return D;
}