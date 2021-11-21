#pragma once
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "Hermite.h"
#include <vector>

#define SPLINE_END 0.9999
#define ERROR_THRESHOLD 0.5

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
		prevTime = 0;
		speed = 0.05;
		return 0;
	};
	int command(int argc, myCONST_SPEC char** argv);
	void registerHermite(Hermite* hermite);
	void setTargetToStart();
	void setupSpline(char* filename);
protected:
	double prevTime;
	double prevTargetT;
	Vector prevTarget, target;
	double speed, speedMultiplier;
	BaseSystem* m_object;
	Hermite* tracedPath;
	// Flags for tracking the current state of simulation
	animationState state;
};
