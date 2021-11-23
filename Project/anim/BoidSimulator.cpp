#include "BoidSimulator.h"
#include "States.h"
#include "QuadTree.h"

BoidSimulator::BoidSimulator(const std::string& name, BaseSystem* target) : BaseSimulator(name), m_object(target) {
	prevTime = 0;
};

void limitVelocity(Boid* b) {
	double speed = VecLength(b->velocity);
	if (speed > b->maxSpeed) {
		VecScale(b->velocity, 1.0 / speed);
		VecScale(b->velocity, b->maxSpeed);
	}
}

void updatePosition(Boid* b, double deltaTime) {
	limitVelocity(b);
	Vector posOffset;
	VecCopy(posOffset, b->velocity);
	VecScale(posOffset, deltaTime);
	VecAdd(b->position, b->position, posOffset);
}

void checkBoundries(Boid* b) {
	for (int i = 0; i < 2; i++) {
		if (b->position[i] < -SCREEN_EDGE + EDGE_MARGIN) {
			b->velocity[i] = b->velocity[i] + 0.05;
		}
		else if (b->position[i] > SCREEN_EDGE - EDGE_MARGIN) {
			b->velocity[i] = b->velocity[i] - 0.05;
		}
	}
}

void BoidSimulator::checkPredatorFood(Boid* p, Flock* flock) {
	for (std::list<Boid*>::iterator it = flock->members.begin(); it != flock->members.end(); ++it) {
		Boid* nextBoid = *it;
		double x = nextBoid->position[0] - p->position[0];
		double y = nextBoid->position[1] - p->position[1];
		if (x*x + y*y < 0.5) {
			p->hasEaten = true;
			BoidSetState state;
			state.toDelete = nextBoid;
			m_object->setState((double*) &state);
			return;
		}
	}
}

int BoidSimulator::step(double time) {
	BoidState* state = new BoidState();
	double deltaTime = time - prevTime;
	if (deltaTime < 0) {
		deltaTime = time;
	}
	m_object->getState((double*)state);

	for (std::list<Boid*>::iterator it = state->predators->members.begin(); it != state->predators->members.end(); ++it) {
		updatePosition(*it, deltaTime);
		checkBoundries(*it);
	}

	for (std::vector<Flock>::iterator itFlock = state->flocks->begin(); itFlock != state->flocks->end(); ++itFlock) {
		Vector center, origin;
		zeroVector(origin);
		
		QuadTree* qTree = new QuadTree("qTree", 12, origin);
		for (Boid* b : itFlock->members) {
			qTree->insert(b);
		}
		quadTrees.push_back(qTree);
		calculateFlockCenter(center, &(*itFlock));
		avoidPredators(&(*itFlock), state->predators, qTree);
		for (std::list<Boid*>::iterator it = itFlock->members.begin(); it != itFlock->members.end(); ++it) {
			Boid* nextBoid = (*it);
			updatePosition(nextBoid, deltaTime);
			Flock closeBoids;
			Circle c = Circle(nextBoid->position[0], nextBoid->position[1], 2);
			qTree->query(&c, closeBoids.members);
			checkBoundries(nextBoid);
			updateDirection(&(*nextBoid), center, &closeBoids);
		}
		qTree->freeChildren();
	}
	m_object->display();
	quadTrees.clear();
	quadTrees.shrink_to_fit();
	prevTime = time;
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
	VecScale(cohesionFactor, 0.0005);
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
	VecScale(alignFactor, 0.01);
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
	VecScale(sepFactor, 0.05);
	VecAdd(desiredVelocity, desiredVelocity, sepFactor);
}

void BoidSimulator::avoidPredators(Flock* normalBirds, Flock* predators, QuadTree* qTree) {
	for (std::list<Boid*>::iterator predatorIt = predators->members.begin(); predatorIt != predators->members.end(); ++predatorIt) {
		Flock closeBoids;
		Circle c = Circle((*predatorIt)->position[0], (*predatorIt)->position[1], 2);
		// Close boids is a subset of boids that is close to the predator
		qTree->query(&c, closeBoids.members);
		Boid* p = *predatorIt;
		checkPredatorFood(p, &closeBoids);
		for (std::list<Boid*>::iterator bIter = closeBoids.members.begin(); bIter != closeBoids.members.end(); ++bIter) {
			Boid* b = *bIter;
			Vector displacement;
			VecSubtract(displacement, b->position, p->position);
			VecLength(displacement);
			VecScale(displacement, 0.5);
			VecAdd(b->velocity, b->velocity, displacement);
			VecNormalize(b->velocity);
		}
	}
}

void BoidSimulator::calculateFlockCenter(Vector center, Flock* flock) {
	zeroVector(center);
	for (std::list<Boid*>::iterator it = flock->members.begin(); it != flock->members.end(); ++it) {
		VecAdd(center, (*it)->position, center);
	}
	VecScale(center, 1.0 / flock->members.size());
}

void BoidSimulator::display(GLenum mode) {
	for (QuadTree* qTree : quadTrees) {
		qTree->display();
	}
}