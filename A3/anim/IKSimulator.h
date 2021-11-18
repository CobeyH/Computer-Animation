#pragma once
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "Hermite.h"
#include <vector>

enum animationState {
	WAITING_FOR_SPLINE,
	GOING_TO_SPLINE_START,
	GOING_ALONG_SPLINE
};

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
	double prevTime;
	double prevTargetT;
	BaseSystem* m_object;
	Hermite* tracedPath;
	// Flags for tracking the current state of simulation
	animationState state;
};
