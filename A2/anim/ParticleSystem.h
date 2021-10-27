#pragma once
#include "BaseSystem.h"
#include "States.h"
#include <vector>

class ParticleSystem : public BaseSystem {
public:
	ParticleSystem(const std::string& name);
	void generateInitalParticles(double numParticles);
	void updateParticle(int index, Vector pos, Vector vel, double mass);
	void updateAllVelocities(Vector velocity);
	void getState(double* p);
	void setState(double* p);
	void reset(double time);
	void display(GLenum mode = GL_RENDER);
	int command(int argc, myCONST_SPEC char** argv);
protected:
	std::vector<Particle> particles;
};

