#pragma once
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "States.h"
#include <vector>

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
	std::vector<Spring> springs;
	int maxSprings;
	BaseSystem* m_object;
	void addSpring(int start, int end, double ks, double kd, double restLength);
};


