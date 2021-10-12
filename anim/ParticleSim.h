#pragma once
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "States.h"
#include <vector>

struct GlobalForces {
	GlobalForces() {
		gravity = 0;
		drag = 0;
		groundForce = 0;
		groundDamp = 0;
	}
	double gravity;
	double drag;
	double groundForce;
	double groundDamp;
};

class ParticleSim : public BaseSimulator {
public:
	ParticleSim(const std::string& name, BaseSystem* target);

	int step(double time);
	int init(double time)
	{
		return 0;
	};

	int command(int argc, myCONST_SPEC char** argv);
protected:
	double prevTime;
	std::vector<Spring> springs;
	GlobalForces* globalForce;
	int maxSprings;
	BaseSystem* m_object;
	void addSpring(int start, int end, double ks, double kd, double restLength);
	void calculateDragForce(Vector dForce);
	void calculateGravityForce(Vector gravForce);
	void calculateSpringForce(Vector sForce);
	void calculateGroundForces(Vector groundForce);
	void calculateNetForces(Particle* p);
};


