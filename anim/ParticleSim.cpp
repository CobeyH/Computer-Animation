#include "ParticleSim.h"
#include "States.h"

ParticleSim::ParticleSim(const std::string& name, BaseSystem* target) : BaseSimulator(name),
m_object(target) {
};

int ParticleSim::step(double time) {
	return -1;
};

void ParticleSim::addSpring(int start, int end, double ks, double kd, double restLength) {
	if (springs.size() >= maxSprings) {
		return;
	}
	Spring* newSpring = new Spring(start, end, ks, kd, restLength);
	springs.push_back(*newSpring);
}

int ParticleSim::command(int argc, myCONST_SPEC char** argv) {
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	
	else if (strcmp(argv[0], "link ") == 0) {
		if (argc != 3) {
			animTcl::OutputMessage("Invalid arguments passed. Expeced 3 arguments but got %d", argc);
			return TCL_ERROR;
		}
	}

	else if (strcmp(argv[0], "spring  ") == 0) {
		if (argc != 6) {
			animTcl::OutputMessage("Invalid arguments passed. Expeced 6 arguments but got %d", argc);
			return TCL_ERROR;
		}
		addSpring(atoi(argv[1]), atoi(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]));
	}

	else if (strcmp(argv[0], "fix") == 0) {
		if (argc != 2) {
			animTcl::OutputMessage("Invalid arguments passed. Expeced 2 arguments but got %d", argc);
			return TCL_ERROR;
		}
		ParticleLock* lock = new ParticleLock();
		lock->shouldLock = true;
		lock->index = atoi(argv[1]);
		m_object->setState((double*) lock);
	}

	else if (strcmp(argv[0], "integration") == 0) {
		if (argc != 3) {
			animTcl::OutputMessage("Invalid arguments passed. Expeced 3 arguments but got %d", argc);
			return TCL_ERROR;
		}
	}

	else if (strcmp(argv[0], "gravity") == 0) {
		if (argc != 2) {
			animTcl::OutputMessage("Invalid arguments passed. Expeced 2 arguments but got %d", argc);
			return TCL_ERROR;
		}
	}

	else if (strcmp(argv[0], "drag") == 0) {
		if (argc != 2) {
			animTcl::OutputMessage("Invalid arguments passed. Expeced 2 arguments but got %d", argc);
			return TCL_ERROR;
		}
	}
};