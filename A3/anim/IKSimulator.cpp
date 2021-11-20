#include "IKSimulator.h"
#include "Hermite.h"
#include "States.h"
#include <util/vectorObj.h>
#include <vector>

IKSimulator::IKSimulator(const std::string& name, BaseSystem* target) : BaseSimulator(name), m_object(target) {
	prevTime = 0;
	state = WAITING_FOR_SPLINE;
	prevTargetT = 0;
};

void IKSimulator::registerHermite(Hermite* hermite) {
	tracedPath = hermite;
}

int IKSimulator::step(double time) {
	if (state == WAITING_FOR_SPLINE) {
		animTcl::OutputMessage("Please load a spline before starting the simulation");
		return TCL_ERROR;
	}
	double deltaTime = time - prevTime;
	Vector effectorPos, target, error, pTarget, pError;
	
	switch (state) {
		case GOING_TO_SPLINE_START:
		{
			zeroVector(target);
			VectorObj targetObj = tracedPath->getIntermediatePoint(0);
			setVector(target, targetObj[0], targetObj[1], targetObj[2]);
			break;
		}
		case GOING_ALONG_SPLINE:
		{
			// need to prevent T from going over 1
			double nextT = min(prevTargetT + 0.0001, 0.9999);
			VectorObj targetObj = tracedPath->getIntermediatePoint(nextT);
			setVector(target, targetObj[0], targetObj[1], targetObj[2]);
			prevTargetT = nextT;
			break;
		}
		default:
			animTcl::OutputMessage("REACHED UNREACHABLE CODE. EXITING");
			return TCL_ERROR;

	}
	m_object->getState(effectorPos);
	VecSubtract(error, target, effectorPos);
	VecCopy(pError, error);
	if (state == GOING_TO_SPLINE_START) {
		VecScale(pError, 0.1);
	}
	
	animTcl::OutputMessage("Error Magnitude %f", VecLength(error));
	VecAdd(pTarget, pError, effectorPos);
	SetBobState* update = new SetBobState();
	update->mode = newTarget;
	VecCopy(update->target, pTarget);
	m_object->setState((double*) update);

	prevTime = time;
	if (state == GOING_TO_SPLINE_START) {
		if (VecLength(error) < ERROR_THRESHOLD) {
			state = GOING_ALONG_SPLINE;
		}
	} else {
		if (prevTargetT == SPLINE_END) {
			state = GOING_TO_SPLINE_START;
			prevTargetT = 0;
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

		glutPostRedisplay();
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