#pragma once
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "States.h"
#include <vector>

enum IntegrationMethod {
	Euler,
	Symplectic,
	Verlet
};

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
		mode = Euler;
		prevTime = time;
		return 0;
	};

	int command(int argc, myCONST_SPEC char** argv);
protected:
	double prevTime;
	GlobalForces* globalForce;
	IntegrationMethod mode;
	int maxSprings;
	int springCount;
	BaseSystem* m_object;
	void addSpring(int start, int end, double ks, double kd, double restLength);
	void calculateDragForce(Vector dForce, Vector velocity);
	void calculateGravityForce(Vector gravForce, double mass);
	void calculateNetSpringForce(Vector sForce, Particle* p);
	void calculateGroundForces(Vector groundForce);
	void calculateNetForces(Vector netForce, Particle* p);
	void setIntegrationMode(char* newMode);
};


