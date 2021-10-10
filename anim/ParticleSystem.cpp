#include "ParticleSystem.h"

ParticleSystem::ParticleSystem(const std::string& name): BaseSystem(name) {
	
};

void ParticleSystem::getState(double* p) {
};

void ParticleSystem::setState(double* p) {
};

void ParticleSystem::reset(double time) {
};

void ParticleSystem::display(GLenum mode) {
};

int ParticleSystem::command(int argc, myCONST_SPEC char** argv) {
	return -1;
};