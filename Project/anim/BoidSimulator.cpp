#include "BoidSimulator.h"
#include "States.h"
#include "QuadTree.h"

BoidSimulator::BoidSimulator(const std::string& name, BaseSystem* target) : BaseSimulator(name), m_object(target) {

};

int BoidSimulator::step(double time) {
	BoidState* state = new BoidState();
	double deltaTime = time - prevTime;
	if (deltaTime < 0) {
		deltaTime = time;
	}
	m_object->getState((double*)state);
	quadTrees.clear();
	for (std::vector<Flock>::iterator itFlock = state->flocks->begin(); itFlock != state->flocks->end(); ++itFlock) {
		Vector posOffset, center, origin;
		zeroVector(origin);

		QuadTree* qTree = new QuadTree("qTree", 12, origin);
		for (Boid* b : itFlock->members) {
			qTree->insert(b);
		}
		quadTrees.push_back(*qTree);
		calculateFlockCenter(center, &(*itFlock));
		for (std::list<Boid*>::iterator it = itFlock->members.begin(); it != itFlock->members.end(); ++it) {
			Boid* nextBoid = (*it);
			VecCopy(posOffset, nextBoid->velocity);
			VecScale(posOffset, nextBoid->speed);
			VecAdd(nextBoid->position, nextBoid->position, posOffset);
			Flock* closeBoids = new Flock();
			Circle* c = new Circle(nextBoid->position[0], nextBoid->position[1], 2);
			qTree->query(*c, closeBoids->members);
			updateDirection(&(*nextBoid), center, closeBoids);
			// Check if the boid has gone out of bounds
			for (int i = 0; i < 3; i++) {
				if (nextBoid->position[i] < -6 ) {
					nextBoid->velocity[i] = abs(nextBoid->velocity[i]);
				}
				else if (nextBoid->position[i] > 6) {
					nextBoid->velocity[i] = -abs(nextBoid->velocity[i]);
				}
			}
		}
	}

	m_object->display();
	time = prevTime;
	return TCL_OK;
}

void BoidSimulator::updateDirection(Boid* b, Vector center, Flock* flock) {
	// Should not calculate velocity for single member flocks.
	if (flock->members.size() <= 1) {
		return;
	}
	Vector steeringForce, desiredVelocity;
	VecCopy(desiredVelocity, b->velocity);
	addCohesion(b, center, desiredVelocity);
	addAlignment(b, flock, desiredVelocity);
	addSeparation(b, flock, desiredVelocity);
	VecSubtract(steeringForce, desiredVelocity, b->velocity);
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
		if ((*it)->id != b->id) {
			VecAdd(alignFactor, alignFactor, (*it)->velocity);
		}
	}
	VecScale(alignFactor, 1.0 / (flock->members.size() - 1));
	VecSubtract(alignFactor, alignFactor, b->velocity);
	VecScale(alignFactor, 1.0 / 300.0);
	VecAdd(desiredVelocity, desiredVelocity, alignFactor);
}
void BoidSimulator::addSeparation(Boid* b, Flock* flock, Vector desiredVelocity) {
	Vector vecBetween, sepFactor;
	zeroVector(sepFactor);
	for (auto it = flock->members.begin(); it != flock->members.end(); ++it) {
		if ((*it)->id != b->id) {
			VecSubtract(vecBetween, (*it)->position, b->position);
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
	for (std::list<Boid*>::iterator it = flock->members.begin(); it != flock->members.end(); ++it) {
		VecAdd(center, (*it)->position, center);
	}
	VecScale(center, 1.0 / flock->members.size());
}

void BoidSimulator::display(GLenum mode) {
	for (QuadTree qTree : quadTrees) {
		qTree.display();
	}
}