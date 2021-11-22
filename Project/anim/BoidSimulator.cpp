#include "BoidSimulator.h"
#include "States.h"

BoidSimulator::BoidSimulator(const std::string& name, BaseSystem* target) : BaseSimulator(name), m_object(target) {

};

int BoidSimulator::step(double time) {
	BoidState* state = new BoidState();
	double deltaTime = time - prevTime;
	if (deltaTime < 0) {
		deltaTime = time;
	}
	m_object->getState((double*)state);
	for (std::vector<Flock>::iterator itFlock = state->flocks->begin(); itFlock != state->flocks->end(); ++itFlock) {
		Vector posOffset, center;
		calculateFlockCenter(center, &(*itFlock));
		for (std::vector<Boid>::iterator it = itFlock->members.begin(); it != itFlock->members.end(); ++it) {
			VecCopy(posOffset, it->velocity);
			VecScale(posOffset, it->speed);
			VecAdd(it->position, it->position, posOffset);
			updateDirection(&(*it), center, (&(*itFlock)));
			// Check if the boid has gone out of bounds
			for (int i = 0; i < 3; i++) {

				if (it->position[i] < -6 ) {
					it->velocity[i] = abs(it->velocity[i]);
				}
				else if (it->position[i] > 6) {
					it->velocity[i] = -abs(it->velocity[i]);
				}
			}
		}
	}

	m_object->display();
	time = prevTime;
	return TCL_OK;
}

void BoidSimulator::updateDirection(Boid* b, Vector center, Flock* flock) {
	Vector steeringForce, desiredVelocity;
	VecCopy(desiredVelocity, b->velocity);
	addCohesion(b, center, desiredVelocity);
	addAlignment(b, flock, desiredVelocity);
	addSeparation(b, flock, desiredVelocity);
	VecSubtract(steeringForce, desiredVelocity, b->velocity);
	if (VecLength(steeringForce) > 0.005) {
		VecNormalize(steeringForce);
		VecScale(steeringForce, 0.005);
	}
	VecAdd(b->velocity, steeringForce, b->velocity);
	VecNormalize(b->velocity);

	double test = VecLength(steeringForce);
	
}

void BoidSimulator::addCohesion(Boid* b, Vector center, Vector desiredVelocity) {
	Vector cohesionFactor;
	zeroVector(cohesionFactor);
	VecSubtract(cohesionFactor, center, b->position);
	VecScale(cohesionFactor, 1.0 / 200);
	VecAdd(desiredVelocity, desiredVelocity, cohesionFactor);

}
void BoidSimulator::addAlignment(Boid* b, Flock* flock, Vector desiredVelocity) {
	Vector alignFactor;
	zeroVector(alignFactor);
	for (auto it = flock->members.begin(); it != flock->members.end(); ++it) {
		if (it->id != b->id) {
			VecAdd(alignFactor, alignFactor, it->velocity);
		}
	}
	VecScale(alignFactor, 1.0 / (flock->members.size() - 1));
	VecSubtract(alignFactor, alignFactor, b->velocity);
	VecScale(alignFactor, 1.0 / 600.0);
	VecAdd(desiredVelocity, desiredVelocity, alignFactor);
}
void BoidSimulator::addSeparation(Boid* b, Flock* flock, Vector desiredVelocity) {
	Vector vecBetween, sepFactor;
	zeroVector(sepFactor);
	for (auto it = flock->members.begin(); it != flock->members.end(); ++it) {
		if (it->id != b->id) {
			VecSubtract(vecBetween, it->position, b->position);
			double distBetween = VecLength(vecBetween);
			if (distBetween < 1) {
				VecSubtract(sepFactor, sepFactor, vecBetween);
			}
		}
	}
	VecScale(sepFactor, 0.01);
	VecAdd(desiredVelocity, desiredVelocity, sepFactor);
}

void BoidSimulator::calculateFlockCenter(Vector center, Flock* flock) {
	zeroVector(center);
	for (std::vector<Boid>::iterator it = flock->members.begin(); it != flock->members.end(); ++it) {
		VecAdd(center, it->position, center);
	}
	VecScale(center, 1.0 / flock->members.size());
}