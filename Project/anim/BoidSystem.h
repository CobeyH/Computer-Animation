#pragma once
#include "BaseSystem.h"
#include "States.h"
#include <vector>

#define FLOCK_COUNT 3

class BoidSystem : public BaseSystem {
public:
	// Inherited and required functions
	BoidSystem(const std::string& name);
	void getState(double* p);
	void setState(double* p);
	void reset(double time);
	void display(GLenum mode = GL_RENDER);
	int command(int argc, myCONST_SPEC char** argv);
	// Extra methods
	void generateInitalBoids(double numBoids);
private:
	std::vector<Flock> flocks;
	Flock* predators;
};

