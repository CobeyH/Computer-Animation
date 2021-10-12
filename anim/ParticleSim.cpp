#include "ParticleSim.h"
#include "States.h"

ParticleSim::ParticleSim(const std::string& name, BaseSystem* target) : BaseSimulator(name),
m_object(target) {
	prevTime = 0;
	globalForce = new GlobalForces();
};

int ParticleSim::step(double time) {
	ParticleState* state = new ParticleState();
	double deltaTime = time - prevTime;
	m_object->getState((double*)state);
	for (auto it = state->particles->begin(); it != state->particles->end(); ++it) {
		if (it->locked) {
			continue;
		}
		Vector moveOffset;
		// Calculate displacement
		VecCopy(moveOffset, it->velocity);
		VecScale(moveOffset, deltaTime);
		// Add displacement to current position
		VecAdd(it->position, it->position, moveOffset);
	}
	m_object->display();
	prevTime = time;
	return -1;
};

void ParticleSim::calculateSpringForce(Vector sForce) {
	zeroVector(sForce);
}

void ParticleSim::calculateDragForce(Vector dForce) {
	zeroVector(dForce);
}

void ParticleSim::calculateGravityForce(Vector gravForce) {
	zeroVector(gravForce);
}

void ParticleSim::calculateGroundForces(Vector groundForce) {
	zeroVector(groundForce);
};

void ParticleSim::calculateNetForces(Particle* p) {
	
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