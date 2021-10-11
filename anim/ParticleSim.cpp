#include "ParticleSim.h"

ParticleSim::ParticleSim(const std::string& name, BaseSystem* target): BaseSimulator(name),
m_object(target) {
};

int ParticleSim::step(double time) {
	return -1;
};

int ParticleSim::command(int argc, myCONST_SPEC char** argv) {
	return -1;
};