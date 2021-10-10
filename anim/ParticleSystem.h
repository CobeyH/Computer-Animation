#pragma once
#include "BaseSystem.h"
#include <vector>

struct Particle {
	Vector position;
	Vector velocity;
};

class ParticleSystem : public BaseSystem {
public:
	ParticleSystem(const std::string& name);
	void getState(double* p);
	void setState(double* p);
	void reset(double time);
	void display(GLenum mode = GL_RENDER);
	int command(int argc, myCONST_SPEC char** argv);
};

