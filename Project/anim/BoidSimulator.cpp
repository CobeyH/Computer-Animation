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

void BoidSimulator::checkPredatorFood(Boid* p, Boid* closeBoids[], int flockSize) {
	for (int i = 0; i < flockSize; i++) {
		Boid* nextBoid = closeBoids[i];
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

void BoidSimulator::updateFlockMembers(Flock* flock, Flock* predators, double deltaTime) {
	Vector center, origin;
	zeroVector(origin);

	QuadTree* qTree = new QuadTree("qTree", 12, origin);
	for (Boid* b : flock->members) {
		qTree->insert(b);
	}
	calculateFlockCenter(center, flock);
	avoidPredators(flock, predators, qTree);
	Boid** foundBoids = (Boid**) malloc(flock->members.size() * sizeof(Boid*));
	for (std::list<Boid*>::iterator it = flock->members.begin(); it != flock->members.end(); ++it) {
		Boid* nextBoid = (*it);
		updatePosition(nextBoid, deltaTime);
		Circle c = Circle(nextBoid->position[0], nextBoid->position[1], 2);
		int flockSize = 0;
		qTree->query(&c, foundBoids, flockSize);
		foundBoids[flockSize] = NULL;
		checkBoundries(nextBoid);
		updateDirection(&(*nextBoid), center, foundBoids, flockSize);
	}
	//free(foundBoids);
	qTree->freeChildren();
}

void BoidSimulator::updateAllBoids(BoidState* state, double deltaTime) {
	// Update Predator Positions
	for (std::list<Boid*>::iterator it = state->predators->members.begin(); it != state->predators->members.end(); ++it) {
		updatePosition(*it, deltaTime);
		checkBoundries(*it);
	}

	
	/*for (std::vector<Flock>::iterator itFlock = state->flocks->begin(); itFlock != state->flocks->end(); ++itFlock) {
	
		updateFlockMembers(&(*itFlock), state->predators, deltaTime);
	}*/



	std::vector<std::thread> t;
	for (std::vector<Flock>::iterator itFlock = state->flocks->begin(); itFlock != state->flocks->end(); ++itFlock) {
		std::thread th = std::thread([this, itFlock, state, deltaTime]() { updateFlockMembers(&(*itFlock), state->predators, deltaTime); });
		t.push_back(std::move(th));  //<=== move (after, th doesn't hold it anymore 
	}

	for (auto& th : t) {              //<=== range-based for uses & reference
		th.join();
	}
}

int BoidSimulator::step(double time) {
	double deltaTime = time - prevTime;
	if (deltaTime < 0) {
		deltaTime = time;
	}
	BoidState* state = new BoidState();
	m_object->getState((double*)state);

	updateAllBoids(state, deltaTime);
	
	m_object->display();
	prevTime = time;
	return TCL_OK;
}

void BoidSimulator::updateDirection(Boid* b, Vector center, Boid* closeBoids[], int size) {
	// Should not calculate velocity for single member flocks.
	if (closeBoids[0] == NULL || closeBoids[1] == NULL) {
		return;
	}
	Vector steeringForce, desiredVelocity;
	VecCopy(desiredVelocity, b->velocity);
	addCohesion(b, center, desiredVelocity);
	addAlignment(b, closeBoids, size, desiredVelocity);
	addSeparation(b, closeBoids, size, desiredVelocity);
	VecSubtract(steeringForce, desiredVelocity, b->velocity);
	VecAdd(b->velocity, steeringForce, b->velocity);
	VecNormalize(b->velocity);
	
}

void BoidSimulator::addCohesion(Boid* b, Vector center, Vector desiredVelocity) {
	Vector cohesionFactor;
	zeroVector(cohesionFactor);
	VecSubtract(cohesionFactor, center, b->position);
	VecScale(cohesionFactor, 0.0005);
	VecAdd(desiredVelocity, desiredVelocity, cohesionFactor);

}

void BoidSimulator::addAlignment(Boid* b, Boid* closeBoids[], int size, Vector desiredVelocity) {
	Vector alignFactor;
	zeroVector(alignFactor);
	for (int i = 0; i < size; i++) {
		if (closeBoids[i]->id != b->id) {
			VecAdd(alignFactor, alignFactor, closeBoids[i]->velocity);
		}
	}
	VecScale(alignFactor, 1.0 / (size - 1));
	VecSubtract(alignFactor, alignFactor, b->velocity);
	VecScale(alignFactor, 0.01);
	VecAdd(desiredVelocity, desiredVelocity, alignFactor);
}

void BoidSimulator::addSeparation(Boid* b, Boid* closeBoids[], int size, Vector desiredVelocity) {
	Vector vecBetween, sepFactor;
	zeroVector(sepFactor);
	for (int i = 0; i < size; i++) {
		if (closeBoids[i]->id != b->id) {
			VecSubtract(vecBetween, closeBoids[i]->position, b->position);
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
	Boid** foundBoids = (Boid**)malloc(normalBirds->members.size() * sizeof(Boid*));
	for (std::list<Boid*>::iterator predatorIt = predators->members.begin(); predatorIt != predators->members.end(); ++predatorIt) {
		Circle c = Circle((*predatorIt)->position[0], (*predatorIt)->position[1], 2);
		// Close boids is a subset of boids that is close to the predator
		int flockSize = 0;
		qTree->query(&c, foundBoids, flockSize);
		Boid* p = *predatorIt;
		checkPredatorFood(p, foundBoids, flockSize);
		for (int i = 0; i < flockSize; i++) {
			Boid* b = foundBoids[i];
			Vector displacement;
			VecSubtract(displacement, b->position, p->position);
			VecLength(displacement);
			VecScale(displacement, 0.5);
			VecAdd(b->velocity, b->velocity, displacement);
			VecNormalize(b->velocity);
		}
	}
	//free(foundBoids);
}

void BoidSimulator::calculateFlockCenter(Vector center, Flock* flock) {
	zeroVector(center);
	for (std::list<Boid*>::iterator it = flock->members.begin(); it != flock->members.end(); ++it) {
		VecAdd(center, (*it)->position, center);
	}
	VecScale(center, 1.0 / flock->members.size());
}

void BoidSimulator::display(GLenum mode) {
	/*for (QuadTree* qTree : quadTrees) {
		qTree->display();
	}*/
}