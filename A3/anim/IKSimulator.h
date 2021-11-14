#pragma once
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "Hermite.h"
#include <vector>

class IKSimulator : public BaseSimulator {
public:
	IKSimulator(const std::string& name, BaseSystem* target);
	int step(double time);
	int init(double time)
	{
		return 0;
	};
	int command(int argc, myCONST_SPEC char** argv);
	void registerHermite(Hermite* hermite);
protected:
	BaseSystem* m_object;
	Hermite* tracedPath;
};
