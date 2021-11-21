#pragma once
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "States.h"
#include <vector>

class BoidSimulator : public BaseSimulator {
public:
	BoidSimulator(const std::string& name, BaseSystem* target);
	int step(double time);
	int init(double time) {
		double prevTime = 0;
		return 0;
	};
protected:
	double prevTime;
	BaseSystem* m_object;
	void calculateFlockCenter(Vector center, BoidState* state);
	void updateDirection(Boid* b, Vector center, BoidState* state);
	void addCohesion(Boid* b, Vector center);
	void addAlignment(Boid* b, BoidState* state);
	void addSeparation(Boid* b, BoidState* state);
};

