#include "ParticleSim.h"
#include "States.h"
#include "GlobalResourceManager.h"
#include <math.h>
#include <iomanip> 

ParticleSim::ParticleSim(const std::string& name, BaseSystem* target) : BaseSimulator(name), m_object(target) {
	globalForce = new GlobalForces();
	springCount = 0;
	maxSprings = 10;
};

int ParticleSim::step(double time) {
	ParticleState* state = new ParticleState();
	double deltaTime = time - prevTime;
	if (deltaTime < 0) {
		deltaTime = time;
	}
	m_object->getState((double*)state);
	// Update particle positions
	for (auto it = state->particles->begin(); it != state->particles->end(); ++it) {
		if (it->locked) {
			continue;
		}
		// Calculate acceleration for next step
		Vector acceleration, moveOffset;
		getAcceleration(deltaTime, acceleration, &(*it));

		switch (mode) {
			case Euler:
				getPositionOffset(deltaTime, moveOffset, &(*it));
				VecAdd(it->position, it->position, moveOffset);
				VecAdd(it->velocity, it->velocity, acceleration);
				break;
			case Symplectic:
				VecAdd(it->velocity, it->velocity, acceleration);
				getPositionOffset(deltaTime, moveOffset, &(*it));
				VecAdd(it->position, it->position, moveOffset);
				break;
			case Verlet:
				Vector forwardPos, velTemp;
				VecCopy(forwardPos, it->position);
				VecScale(forwardPos, 2);
				VecSubtract(forwardPos, forwardPos, it->prevPos);
				VecScale(acceleration, deltaTime*deltaTime);
				VecAdd(forwardPos, forwardPos, acceleration);
				VecCopy(it->prevPos, it->position);
				VecCopy(it->position, forwardPos);

				VecSubtract(velTemp, forwardPos, it->prevPos);
				VecScale(velTemp, 1.0/ (2 * deltaTime));
				VecCopy(it->velocity, velTemp);
				break;
		}
	}
	m_object->display();
	prevTime = time;
	return -1;
};

void ParticleSim::getPositionOffset(double deltaTime, Vector moveOffset, Particle* p) {
	VecCopy(moveOffset, p->velocity);
	VecScale(moveOffset, deltaTime);
}

void ParticleSim::getAcceleration(double deltaTime, Vector acceleration, Particle* p) {
	Vector netForce;
	calculateNetForces(netForce, p);
	VecCopy(acceleration, netForce);
	VecScale(acceleration, 1 / p->mass);
	// Update velocity from acceleration
	VecScale(acceleration, deltaTime);
}

void ParticleSim::calculateNetSpringForce(Vector netSpringForce, Particle* p) {
	zeroVector(netSpringForce);
	for (Spring s : p->connectedSprings) {
		Vector isubj, unitVec, springVec;
		VecSubtract(isubj, p->position, s.endPoint->position);
		double vecLength = VecLength(isubj);
		VecCopy(unitVec, isubj);
		VecScale(unitVec, 1.0/vecLength);
		VecCopy(springVec, unitVec);
		// Spring Force
		double lengthDiff = s.restLength - vecLength;
		// Account for double precision
		if (lengthDiff < 0.000001) {
			continue;
		}
		VecScale(springVec, lengthDiff);
		VecScale(springVec, s.ks);
		VecAdd(netSpringForce, netSpringForce, springVec);
		// Spring Damping
		Vector velDiff, dampVec;
		VecSubtract(velDiff, p->velocity, s.endPoint->velocity);
		double velScalar = VecDotProd(velDiff, unitVec);
		velScalar *= -s.kd;
		VecCopy(dampVec, unitVec);
		VecScale(dampVec, velScalar);
		VecAdd(netSpringForce, netSpringForce, dampVec);
	}
}

void ParticleSim::calculateDragForce(Vector dForce, Vector velocity) {
	VecCopy(dForce, velocity);
	VecScale(dForce, -globalForce->drag);
}

void ParticleSim::calculateGravityForce(Vector gravForce, double mass) {
	double gravScalar = -globalForce->gravity * mass;
	setVector(gravForce, 0, -gravScalar, 0);
}

void ParticleSim::calculateGroundForces(Vector groundForce, Particle* p) {
	Vector P, N, xToP;
	zeroVector(P);
	setVector(N, 0, -1, 0);
	VecSubtract(xToP, p->position, P);
	if (VecDotProd(xToP, N) < 0) {
		return;
	}
	// Calculate force pushing out of the ground
	Vector outwardForce, dampForce;
	VecCopy(outwardForce, N);
	double forceScalar = -globalForce->groundForce * VecDotProd(xToP, N);
	VecScale(outwardForce, forceScalar);
	// Calculate damping force of spring
	VecCopy(dampForce, N);
	forceScalar = -globalForce->groundDamp * VecDotProd(p->velocity, N);
	VecScale(dampForce, forceScalar);
	// Add everything together
	VecAdd(groundForce, groundForce, outwardForce);
	VecAdd(groundForce, groundForce, dampForce);


};

void ParticleSim::calculateNetForces(Vector netForce, Particle* p) {
	Vector sForce, dForce, gravForce, groundForce;
	zeroVector(netForce); zeroVector(sForce); zeroVector(dForce); zeroVector(groundForce);
	// Gravity
	calculateGravityForce(gravForce, p->mass);
	VecAdd(netForce, netForce, gravForce);
	// Ground
	calculateGroundForces(groundForce, p);
	VecAdd(netForce, netForce, groundForce);
	// Drag Force
	calculateDragForce(dForce, p->velocity);
	VecAdd(netForce, netForce, dForce);
	// Spring forces
	calculateNetSpringForce(sForce, p);
	VecAdd(netForce, netForce, sForce);
};

void ParticleSim::addSpring(int start, int end, double ks, double kd, double restLength) {
	if (springCount >= maxSprings) {
		animTcl::OutputMessage("Cannot add a new spring. Spring limit has been reached");
		return;
	}
	ParticleState* state = new ParticleState();
	m_object->getState((double*)state);
	
	Spring* spring1 = new Spring(&state->particles->at(end), ks, kd, restLength);
	UpdateState* springUpdate = new UpdateState(NewSpring);
	springUpdate->spring = spring1;
	springUpdate->index = start;
	m_object->setState((double*)springUpdate);

	Spring* spring2 = new Spring(&state->particles->at(start), ks, kd, restLength);
	springUpdate->spring = spring2;
	springUpdate->index = end;
	m_object->setState((double*)springUpdate);
}

int ParticleSim::command(int argc, myCONST_SPEC char** argv) {
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	
	else if (strcmp(argv[0], "link") == 0) {
		if (argc != 3) {
			animTcl::OutputMessage("Invalid arguments passed. Expeced 3 arguments but got %d", argc);
			return TCL_ERROR;
		}
		BaseSystem* linkedSys = GlobalResourceManager::use()->getSystem(argv[1]);
		assert(linkedSys);
		m_object = linkedSys;
		maxSprings = atoi(argv[2]);
	}

	else if (strcmp(argv[0], "spring") == 0) {
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
		UpdateState* lock = new UpdateState(Lock);
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
		GlobalResourceManager::use()->setNewTimestep(atof(argv[2]));
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
	else if (strcmp(argv[0], "ground") == 0) {
		if (argc != 3) {
			animTcl::OutputMessage("Invalid arguments passed. Expeced 3 arguments but got %d", argc);
			return TCL_ERROR;
		}
		globalForce->groundForce = atof(argv[1]);
		globalForce->groundDamp = atof(argv[2]);
	}
	else {
		animTcl::OutputMessage("The given command %s is not valid", argv[1]);
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