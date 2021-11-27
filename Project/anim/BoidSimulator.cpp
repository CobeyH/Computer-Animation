#include "BoidSimulator.h"
#include "States.h"
#include "QuadTree.h"

BoidSimulator::BoidSimulator(const std::string& name, BaseSystem* target) : BaseSimulator(name), m_object(target) {
	prevTime = 0;
	Vector origin;
	zeroVector(origin);
	foodQTree = new QuadTree<Food>("qTree", 12, origin);
	std::ofstream myFile;
	myFile.open("boidAttrib.csv");
	myFile << "time, speed, alignment, separation, cohesion\n";
};

void limitVelocity(Boid* b) {
	double speed = VecLength(b->velocity);
	if (speed > b->attrib.maxSpeed) {
		VecScale(b->velocity, 1.0 / speed);
		VecScale(b->velocity, b->attrib.maxSpeed);
	}
}

void BoidSimulator::updatePosition(Boid* b, double deltaTime) {
	if (!b->isPredator) {
		b->hunger++;
		if (b->hunger >= STARVATION) {
			starvedBoids.push_back(b);
			return;
		}
	}
	else {
		b->hunger += 5;
	}
	
	
	limitVelocity(b);
	Vector posOffset;
	VecCopy(posOffset, b->velocity);
	VecScale(posOffset, deltaTime);
	VecAdd(b->position, b->position, posOffset);
}

void checkBoundries(Boid* b) {
	for (int i = 0; i < 2; i++) {
		if (b->position[i] < -SCREEN_EDGE ) {
			b->position[i] +=  SCREEN_WIDTH;
		}
		else if (b->position[i] > SCREEN_EDGE) {
			b->position[i] -= SCREEN_WIDTH;
		}
	}
}

void BoidSimulator::killBoid(Boid* b) {
	BoidSetState state;
	state.toDelete = b;
	m_object->setState((double*)&state);
}

void BoidSimulator::checkPredatorFood(Boid* p, Boid* closeBoids[], int flockSize) {
	// Predators will not hunt if they are not hungry
	if (p->hunger < PREDATOR_STARVATION_THREASHOLD * STARVATION) {
		return;
	}
	if (flockSize <= 0) {
		return;
	}
	double closestBoidDistance = INT_MAX;
	int closestBoidIndex = 0;
	for (int i = 0; i < flockSize; i++) {
		Boid* nextBoid = closeBoids[i];
		double x = nextBoid->position[0] - p->position[0];
		double y = nextBoid->position[1] - p->position[1];
		double distanceToBoid = x * x + y * y;
		if (distanceToBoid < PREDATOR_INFLUENCE_RANGE) {
			p->hunger = 0;
			killBoid(nextBoid);
			return;
		}
		if (distanceToBoid < closestBoidDistance) {
			closestBoidDistance = distanceToBoid;
			closestBoidIndex = i;
		}
	}
	Vector dirToBoid;
	VecSubtract(dirToBoid, closeBoids[closestBoidIndex]->position, p->position);
	VecNormalize(dirToBoid);
	VecScale(dirToBoid, 0.05);
	VecAdd(p->velocity, p->velocity, dirToBoid);
}

void BoidSimulator::updateFlockMembers(Flock* flock, Flock* predators, double deltaTime) {
	Vector center, origin;
	zeroVector(origin);

	QuadTree<Boid>* qTree = new QuadTree<Boid>("qTree", 12, origin);
	for (Boid* b : flock->members) {
		qTree->insert(b);
	}
	calculateFlockCenter(center, flock);
	avoidPredators(flock, predators, qTree);
	 
	Boid** foundBoids = (Boid**) malloc((flock->members.size() + 1) * sizeof(Boid*));
	for (std::list<Boid*>::iterator it = flock->members.begin(); it != flock->members.end(); ++it) {
		Boid* nextBoid = (*it);
		updatePosition(nextBoid, deltaTime);
		int flockSize = 0;
		
		Circle c = Circle(nextBoid->position[0], nextBoid->position[1], BOID_PERCEPTION_RANGE);
		qTree->query(&c, foundBoids, flockSize);
			
		
		checkBoundries(nextBoid);
		updateDirection(&(*nextBoid), center, foundBoids, flockSize);
		
	}
	qTree->freeChildren();
	free(foundBoids);
}

void BoidSimulator::updateAllBoids(BoidState* state, double deltaTime) {
	// Update Predator Positions
	for (std::list<Boid*>::iterator it = state->predators->members.begin(); it != state->predators->members.end(); ++it) {
		updatePosition(*it, deltaTime);
		checkBoundries(*it);
	}

	
	//for (std::vector<Flock>::iterator itFlock = state->flocks->begin(); itFlock != state->flocks->end(); ++itFlock) {
	//	updateFlockMembers(&(*itFlock), state->predators, deltaTime);
	//}

	std::vector<std::thread> t;
	for (std::vector<Flock>::iterator itFlock = state->flocks->begin(); itFlock != state->flocks->end(); ++itFlock) {
		std::thread th = std::thread([this, itFlock, state, deltaTime]() { updateFlockMembers(&(*itFlock), state->predators, deltaTime); });
		t.push_back(std::move(th));
	}

	for (auto& th : t) {
		th.join();
	}
}

void BoidSimulator::printInfo(BoidState* state, double time) {
	if (time < prevPrint + PRINT_INTERVAL) {
		return;
	}
	prevPrint = time;
	int numBoids = 0; double averageSpeed = 0; double averageAlign = 0; double averageSep = 0; double averageCoh = 0;
	for (std::vector<Flock>::iterator it = state->flocks->begin(); it != state->flocks->end(); ++it) {
		for (std::list<Boid*>::iterator bIt = it->members.begin(); bIt != it->members.end(); ++bIt) {
			numBoids++;
			averageSpeed += (*bIt)->attrib.maxSpeed;
			averageAlign += (*bIt)->attrib.align;
			averageSep += (*bIt)->attrib.separation;
			averageCoh += (*bIt)->attrib.cohesion;
		}
	}
	averageSpeed /= numBoids;
	averageAlign /= numBoids;
	averageSep /= numBoids;
	averageCoh /= numBoids;
	std::ofstream myFile;
	myFile.open("boidAttrib.csv", std::ios_base::app);
	myFile << time << "," << averageSpeed << "," << averageAlign << "," << averageSep << "," << averageCoh << "\n";
}

int BoidSimulator::step(double time) {
	double deltaTime = time - prevTime;
	if (deltaTime < 0) {
		deltaTime = time;
	}
	
	starvedBoids.clear();
	BoidState* state = new BoidState();
	m_object->getState((double*)state);
	printInfo(state, time);
	updateAllBoids(state, deltaTime);
	for (auto it = starvedBoids.begin(); it != starvedBoids.end(); ++it) {
		killBoid(*it);
	}
	
	m_object->display();
	delete(state);
	prevTime = time;
	return TCL_OK;

}

void BoidSimulator::updateDirection(Boid* b, Vector center, Boid* closeBoids[], int size) {
	Vector steeringForce;
	zeroVector(steeringForce);
	
	// Should not calculate flocking behaviour for single member flocks.
	if (size > 1) {
		addCohesion(b, center, steeringForce);
		addAlignment(b, closeBoids, size, steeringForce);
		addSeparation(b, closeBoids, size, steeringForce);
	}
	if (b->hunger > STARVATION * BOID_STARVATION_THREASHOLD) {
		addFoodAttraction(b, steeringForce);
	}
	VecScale(steeringForce, TURNING_RATE);
	VecAdd(b->velocity, steeringForce, b->velocity);
	VecNormalize(b->velocity);
	
}

void BoidSimulator::addCohesion(Boid* b, Vector center, Vector desiredVelocity) {
	Vector cohesionFactor;
	zeroVector(cohesionFactor);
	VecSubtract(cohesionFactor, center, b->position);
	VecScale(cohesionFactor, COHESION_STRENGTH * b->attrib.cohesion);
	VecAdd(desiredVelocity, desiredVelocity, cohesionFactor);

}

void BoidSimulator::addAlignment(Boid* b, Boid* closeBoids[], int size, Vector desiredVelocity) {
	Vector alignFactor;
	zeroVector(alignFactor);
	for (int i = 0; i < size; i++) {
		if (closeBoids[i]->id != b->id) {
			alignFactor[0] += closeBoids[i]->velocity[0];
			alignFactor[1] += closeBoids[i]->velocity[1];
			//VecAdd(alignFactor, alignFactor, closeBoids[i]->velocity);
		}
	}
	VecScale(alignFactor, 1.0 / (size - 1));
	VecSubtract(alignFactor, alignFactor, b->velocity);
	VecScale(alignFactor, ALIGNMENT_STRENGTH * b->attrib.align);
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
				// Normalize then divide by distance between so that closer objects repell more.
				VecScale(vecBetween, 1 / (distBetween * distBetween));
				VecSubtract(sepFactor, sepFactor, vecBetween);
			}
		}
	}
	VecScale(sepFactor, SEPARATION_STRENGTH * b->attrib.separation);
	VecAdd(desiredVelocity, desiredVelocity, sepFactor);
}

void BoidSimulator::addFoodAttraction(Boid* b, Vector desiredVelocity) {
	// Boids that are mostly full don't care about food
	Food** foundFood = (Food**)malloc(50 * sizeof(Food*)); // TODO: this is the leak
	Circle c = Circle(b->position[0], b->position[1], 2);
	int nearbyFoodCount = 0;
	foodQTree->query(&c, foundFood, nearbyFoodCount);
	if (nearbyFoodCount == 0) {
		free(foundFood);
		return;
	}
	double nearestFoodDist = INT_MAX;
	int nearestFoodIndex = 0;
	for (int i = 0; i < nearbyFoodCount; i++) {
		double distToNextFood = abs(foundFood[i]->position[0] - b->position[0]) + abs(foundFood[i]->position[1] - b->position[1]);
		if (distToNextFood < nearestFoodDist) {
			nearestFoodIndex = i;
			nearestFoodDist = distToNextFood;
		}
	}
	// TODO: Possible performance improvement. Use the first found food instead of the closest.
	Vector dirToFood;
	VecSubtract(dirToFood, foundFood[nearestFoodIndex]->position, b->position);
	double distToFood = VecLength(dirToFood);
	if (distToFood < BOID_EATING_DISTANCE) {
		b->hunger = 0;
	}
	else {
		VecScale(dirToFood, 1.0 / distToFood);
		double foodScalar = pow(b->hunger / (double)STARVATION, 2);
		VecScale(dirToFood, foodScalar);
		VecScale(dirToFood, FOOD_ATTRACTION_STRENGTH);
		VecAdd(desiredVelocity, desiredVelocity, dirToFood);
	}
	free(foundFood);
}

void BoidSimulator::avoidPredators(Flock* normalBirds, Flock* predators, QuadTree<Boid>* qTree) {
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
			VecScale(displacement, PREDATOR_AVOIDANCE_STRENGTH);
			VecAdd(b->velocity, b->velocity, displacement);
			VecNormalize(b->velocity);
		}
	}
	free(foundBoids);
}

void BoidSimulator::calculateFlockCenter(Vector center, Flock* flock) {
	zeroVector(center);
	for (std::list<Boid*>::iterator it = flock->members.begin(); it != flock->members.end(); ++it) {
		VecAdd(center, (*it)->position, center);
	}
	VecScale(center, 1.0 / flock->members.size());
}

void BoidSimulator::registerFoodSystem(BaseSystem* system) {
	foodSystem = system;
	GetFoodState* state = new GetFoodState();
	foodSystem->getState((double*) state);
	for (int i = 0; i < state->foodQuantity; i++) {
		foodQTree->insert(state->food[i]);
	}
	delete(state);
}

void BoidSimulator::display(GLenum mode) {
	//if (temp != NULL) {
	//	glPointSize(10);
	//	glBegin(GL_POINTS);
	//	set_colour(1, 0, 0);
	//	glVertex3dv(temp->position);
	//	glEnd();
	//}
	//if (prevTime > 0) {
	//	temp->display();
	//}

	
}