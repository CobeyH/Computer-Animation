#pragma once
#include "BaseSimulator.h"
#include "BaseSystem.h"
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
	BaseSystem* m_object;
};


