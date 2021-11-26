#include "BoidSystem.h"
#include "BaseSystem.h"
#include <vector>
#include <random>

BoidSystem::BoidSystem(const std::string& name) : BaseSystem(name) {
	for (int i = 0; i < FLOCK_COUNT; i++) {
		Flock* newFlock = new Flock();
		flocks.push_back(*newFlock);
	}
	predators = new Flock();
};

int randNum(int min, int max) {
	if (min == max) {
		return min;
	}
	return min + (rand()) / ((RAND_MAX / (max - min)));
}

double randNum(double min, double max) {
	return min + (double)(rand()) / ((double)(RAND_MAX / (max - min)));
}

void getRandomDirection(Vector dir) {
	double x = rand() % 100 - 50;
	double y = rand() % 100 - 50;
	//int z = rand() % 100 - 50;
	double z = 0;
	setVector(dir, x, y, z);
	VecNormalize(dir);
}

void getRandomPosition(Vector pos) {
	// Generate random numbers from -5 to 5
	double x = randNum(-5.0, 5.0);
	double y = randNum(-5.0, 5.0);
	//int z = (double)(rand() % 800) / 100 - 4;
	int z = 0;
	setVector(pos, x, y, z);
}

void BoidSystem::generateInitalBoids(double numBoids) {
	for (int i = 0; i < flocks.size(); i++) {
		flocks[i].members.clear();
	}
	// Create Normal Boids
	for (int flockIndex = 0; flockIndex < FLOCK_COUNT; flockIndex++) {
		double flockSpeed = randNum(0.5, 1.5);
		double cohesion = randNum(0.5, 1.5);
		double alignment = randNum(0.5, 1.5);
		double separation = randNum(0.5, 1.5);
		double mass = randNum(1, 10);
		for (int i = 0; i < numBoids / FLOCK_COUNT; i++) {
			Vector initalPosition, initalDirection;
			getRandomPosition(initalPosition);
			getRandomDirection(initalDirection);
			Boid* newBoid = new Boid(initalPosition, initalDirection, i, flockIndex, flockSpeed);
			newBoid->attrib.align = alignment;
			newBoid->attrib.cohesion = cohesion;
			newBoid->attrib.separation = separation;
			newBoid->attrib.mass = mass;
			flocks[flockIndex].members.push_back(newBoid);
		}
	}
	// Create Predators
	int numPredators = min(numBoids / 200 + 1, 3);
	for (int i = 0; i < numPredators; i++) {
		Vector initalPosition, initalDirection;
		getRandomPosition(initalPosition);
		getRandomDirection(initalDirection);
		Boid* newBoid = new Boid(initalPosition, initalDirection, i, i % FLOCK_COUNT, true);
		predators->members.push_back(newBoid);
	}
	glutPostRedisplay();
};

void BoidSystem::addEvolvedBoid(int flockMissingBoid) {
	int fIndex1 = randNum(0, FLOCK_COUNT - 1);
	int fIndex2 = randNum(0, FLOCK_COUNT - 1);
	Flock* flock1 = &flocks.at(fIndex1);
	Flock* flock2 = &flocks.at(fIndex2);

	int bIndex1 = randNum(0, flock1->members.size() - 2);
	int bIndex2 = randNum(0, flock2->members.size() - 2);
	auto it1 = flock1->members.begin();
	auto it2 = flock2->members.begin();

	std::advance(it1, bIndex1);
	std::advance(it2, bIndex2);
	Boid* b1 = *it1;
	Boid* b2 = *it2;

	Vector position, rotation;
	setVector(position, -1, 0, 0);
	zeroVector(rotation);

	Boid* child = new Boid(position, rotation, flocks.at(flockMissingBoid).members.size(), flockMissingBoid, 0);

	std::random_device rd;
	std::mt19937 rng(rd());
	std::uniform_int_distribution<int> gen(0, 1);

	int chosenMass = gen(rng);
	child->attrib.mass = chosenMass ? b1->attrib.mass : b2->attrib.mass;
	int chosenCohesion = gen(rng);
	child->attrib.cohesion = chosenCohesion ? b1->attrib.cohesion : b2->attrib.cohesion;
	int chosenSeparation = gen(rng);
	child->attrib.separation = chosenSeparation ? b1->attrib.separation : b2->attrib.separation;
	int chosenAlignment = gen(rng);
	child->attrib.align = chosenAlignment ? b1->attrib.align : b2->attrib.align;
	int chosenSpeed = gen(rng);
	child->attrib.maxSpeed = chosenSpeed ? b1->attrib.maxSpeed : b2->attrib.maxSpeed;
	flocks.at(flockMissingBoid).members.push_back(child);


}

void BoidSystem::getState(double* p) {
	BoidState* state = (BoidState*) p;
	state->flocks = &flocks;
	state->predators = predators;
};

void BoidSystem::setState(double* p) {
	BoidSetState* state = (BoidSetState*) p;
	if (state->toDelete->isPredator) {
		return;
	}
	int flockId = state->toDelete->flockId;
	flocks[flockId].members.remove(state->toDelete);
	addEvolvedBoid(flockId);
}

void BoidSystem::reset(double time) {
	for (int i = 0; i < flocks.size(); i++) {
		// for (int j = 0; j < flocks[i].members.size(); j++) {
		for (std::list<Boid*>::iterator iter = flocks[i].members.begin(); iter != flocks[i].members.end(); ++i) {
			VecCopy((*iter)->position, (*iter)->initalPosition);
		}
	}
}

void drawBoid(Boid* b) {
	Vector  direction, tail, head, tailOffset, leftWing, rightWing, wingOffset, zAxis;
	setVector(zAxis, 0, 0, 1);
	glBegin(GL_TRIANGLE_FAN);
	// Calculate vector from head to tail
	VecCopy(direction, b->velocity);
	VecNormalize(direction);
	VecCopy(tailOffset, direction);
	VecScale(tailOffset, -0.075);
	// Calculate vector orthoginal to that vector
	VecCrossProd(wingOffset, zAxis, tailOffset);
	VecNormalize(wingOffset);
	VecScale(wingOffset, 0.075);


	// Calculate tail position
	VecAdd(tail, b->position, tailOffset);
	VecSubtract(head, b->position, tailOffset);
	// Calculate right wing position
	VecAdd(rightWing, tail, wingOffset);
	VecScale(tailOffset, 0.4);
	VecAdd(rightWing, tailOffset, rightWing);

	// Calculate left wing position
	VecSubtract(leftWing, tail, wingOffset);
	VecAdd(leftWing, tailOffset, leftWing);
	// Draw the boids
	glVertex3dv(head);
	glVertex3dv(leftWing);
	glVertex3dv(tail);
	glVertex3dv(rightWing);
	glEnd();
}

void BoidSystem::display(GLenum mode) {
	glPointSize(3);
	set_colour(0.0, 1.0, 0.0);
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	
	
	for (int i = 0; i < flocks.size(); i++) {
		switch (i) {
		case 0:
			set_colour(1, 1, 0);
			break;
		case 1:
			set_colour(0, 1, 0);
			break;
		case 2:
			set_colour(0, 1, 1);
			break;
		case 3:
			set_colour(1, 0, 0);
			break;
		case 4:
			set_colour(1, 0, 1);
			break;
		default:
			set_colour(1, 1, 1);
		}
		for (Boid* b : flocks[i].members) {
			drawBoid(b);
		}
	}
	set_colour(0, 0, 0);
	for (std::list<Boid*>::iterator it = predators->members.begin(); it != predators->members.end(); ++it) {
		drawBoid((*it));
	}
	glPopAttrib();
}

int BoidSystem::command(int argc, myCONST_SPEC char** argv) {
	if (argc < 1) {
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "dim") == 0) {
		if (argc != 2) {
			animTcl::OutputMessage("Invalid arguments passed. Expeced 9 arguments but got %d", argc);
			return TCL_ERROR;
		}
		generateInitalBoids(atof(argv[1]));
	}
	return TCL_OK;
}