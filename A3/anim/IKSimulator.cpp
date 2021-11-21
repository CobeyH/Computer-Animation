#include "IKSimulator.h"
#include "Hermite.h"
#include "States.h"
#include <util/vectorObj.h>
#include <vector>

IKSimulator::IKSimulator(const std::string& name, BaseSystem* target) : BaseSimulator(name), m_object(target) {
	prevTime = 0;
	state = WAITING_FOR_SPLINE;
	prevTargetT = 0;
	speedMultiplier = 1;
};

void IKSimulator::registerHermite(Hermite* hermite) {
	tracedPath = hermite;
}

void IKSimulator::setTargetToStart() {
	VectorObj targetObj = tracedPath->getIntermediatePoint(0);
	setVector(target, targetObj[0], targetObj[1], targetObj[2]);
	Vector effectorPos;
	m_object->getState(effectorPos);
	VecCopy(prevTarget, effectorPos);
}

int IKSimulator::step(double time) {
	if (state == WAITING_FOR_SPLINE) {
		animTcl::OutputMessage("Please load a spline before starting the simulation");
		return TCL_ERROR;
	}
	double deltaTime = time - prevTime;
	Vector effectorPos, error, pTarget, pError;
	
	switch (state) {
		case GOING_TO_SPLINE_START:
		{
			Vector dir;
			VecSubtract(dir, target, prevTarget);
			VecNormalize(dir);
			VecScale(dir, speed * speedMultiplier);
			VecAdd(pTarget, prevTarget, dir);
			VecCopy(prevTarget, pTarget);
			break;
		}
		case GOING_ALONG_SPLINE:
		{
			// need to prevent T from going over 1
			double nextT = min(prevTargetT + 0.0002 * speedMultiplier, 0.9999);
			VectorObj targetObj = tracedPath->getIntermediatePoint(nextT);
			setVector(target, targetObj[0], targetObj[1], targetObj[2]);
			VecCopy(pTarget, target);
			prevTargetT = nextT;
			break;
		}
		default:
			animTcl::OutputMessage("REACHED UNREACHABLE CODE. EXITING");
			return TCL_ERROR;

	}
	int iterations = 0;
	do {
		iterations++;
		m_object->getState(effectorPos);
		VecSubtract(error, pTarget, effectorPos);
		SetBobState* update = new SetBobState();
		update->mode = newTarget;
		VecCopy(update->target, pTarget);
		m_object->setState((double*)update);
		if (iterations >= 75) {
			// Prevent getting stuck
			break;
		}
	} while (VecLength(error) > 0.5);

	prevTime = time;
	if (state == GOING_TO_SPLINE_START) {
		Vector distanceToStart;
		VecSubtract(distanceToStart, target, effectorPos);
		if (VecLength(distanceToStart) < ERROR_THRESHOLD) {
			state = GOING_ALONG_SPLINE;
		}
	} else {
		if (prevTargetT >= SPLINE_END) {
			state = GOING_TO_SPLINE_START;
			prevTargetT = 0;
			setTargetToStart();
		}
	}
	return TCL_OK;
};

int IKSimulator::command(int argc, myCONST_SPEC char** argv) {
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}

	else if (strcmp(argv[0], "read") == 0) {
		if (argc != 2) {
			animTcl::OutputMessage("Invalid arguments passed. Expeced 2 arguments but got %d", argc);
			return TCL_ERROR;
		}
		setupSpline(argv[1]);
		setTargetToStart();
		glutPostRedisplay();
	}
	else if (strcmp(argv[0], "setSpeed") == 0) {
		if (argc != 2) {
			animTcl::OutputMessage("Invalid arguments passed. Expeced 2 arguments but got %d", argc);
			return TCL_ERROR;
		}
		speedMultiplier = atof(argv[1]);
		
	}
	else {
		animTcl::OutputMessage("The given command %s is not valid", argv[0]);
	}
};

void IKSimulator::setupSpline(char* filename) {
	tracedPath->loadFromFile2D(filename);
	state = GOING_TO_SPLINE_START;
	SetBobState* setupState = new SetBobState();
	setupState->mode = startup;
	m_object->setState((double*) setupState);
}