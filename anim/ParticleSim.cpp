#include "ParticleSim.h"
#include "States.h"

ParticleSim::ParticleSim(const std::string& name, BaseSystem* target) : BaseSimulator(name),
m_object(target) {
	globalForce = new GlobalForces();
};

int ParticleSim::step(double time) {
	ParticleState* state = new ParticleState();
	double deltaTime = time - prevTime;
	// TODO: Fix this weird behaviour when the simulator resets.
	if (deltaTime < 0) {
		deltaTime = time;
	}
	m_object->getState((double*)state);
	// Update particle positions
	for (auto it = state->particles->begin(); it != state->particles->end(); ++it) {
		if (it->locked) {
			continue;
		}
		// Calculate displacement
		Vector moveOffset;
		VecCopy(moveOffset, it->velocity);
		VecScale(moveOffset, deltaTime);
		// Add displacement to current position
		VecAdd(it->position, it->position, moveOffset);

		// Calculate acceleration for next step
		Vector acceleration, netForce;
		double index = it - state->particles->begin();
		calculateNetForces(netForce, &(*it), index);
		VecCopy(acceleration, netForce);
		VecScale(acceleration, 1 / it->mass);
		// Update velocity from acceleration
		VecScale(acceleration, deltaTime);
		VecAdd(it->velocity, it->velocity, acceleration);
	}
	m_object->display();
	prevTime = time;
	return -1;
};

void ParticleSim::calculateSpringForce(Vector sForce, int index) {

}

void ParticleSim::calculateDragForce(Vector dForce, Vector velocity) {
	VecCopy(dForce, velocity);
	VecScale(dForce, -globalForce->drag);
}

void ParticleSim::calculateGravityForce(Vector gravForce, double mass) {
	double gravScalar = globalForce->gravity * mass;
	setVector(gravForce, 0, -gravScalar, 0);
}

void ParticleSim::calculateGroundForces(Vector groundForce) {
	
};

void ParticleSim::calculateNetForces(Vector netForce, Particle* p, int index) {
	Vector sForce, dForce, gravForce, groundForce;
	zeroVector(netForce); zeroVector(sForce); zeroVector(dForce); zeroVector(groundForce);
	// Gravity
	calculateGravityForce(gravForce, p->mass);
	VecAdd(netForce, netForce, gravForce);
	// Ground
	calculateGroundForces(groundForce);
	VecAdd(netForce, netForce, groundForce);
	// Drag Force
	calculateDragForce(dForce, p->velocity);
	VecAdd(netForce, netForce, dForce);
	// Spring forces
	calculateSpringForce(sForce, index);
	VecAdd(netForce, netForce, sForce);
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
		maxSprings = atoi(argv[3]);
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
		setIntegrationMode(argv[1]);
	}

	else if (strcmp(argv[0], "gravity") == 0) {
		if (argc != 2) {
			animTcl::OutputMessage("Invalid arguments passed. Expeced 2 arguments but got %d", argc);
			return TCL_ERROR;
		}
		globalForce->gravity = atof(argv[1]);
	}

	else if (strcmp(argv[0], "drag") == 0) {
		if (argc != 2) {
			animTcl::OutputMessage("Invalid arguments passed. Expeced 2 arguments but got %d", argc);
			return TCL_ERROR;
		}
		globalForce->drag = atof(argv[1]);
	}
};

void ParticleSim::setIntegrationMode(char* newMode) {
	if (strcmp(newMode, "euler") == 0) {
		mode = Euler;
	}
	else if (strcmp(newMode, "symplectic") == 0) {
		mode = Symplectic;
	}
	else if (strcmp(newMode, "verlet") == 0) {
		mode = Verlet;
	}
	else {
		animTcl::OutputMessage("Unsupported integration mode provided");
		return;
	}
	animTcl::OutputMessage("Integration mode set to %s", newMode);
}