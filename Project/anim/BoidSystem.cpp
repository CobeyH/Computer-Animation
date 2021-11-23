#include "BoidSystem.h"
#include "BaseSystem.h"
#include <vector>

BoidSystem::BoidSystem(const std::string& name) : BaseSystem(name) {
	for (int i = 0; i < FLOCK_COUNT; i++) {
		Flock* newFlock = new Flock();
		flocks.push_back(*newFlock);
	}
};

void getRandomDirection(Vector dir) {
	double x = rand() % 100 - 50;
	double y = rand() % 100 - 50;
	//int z = rand() % 100 - 50;
	double z = 0;
	setVector(dir, x, y, z);
	VecNormalize(dir);
}

void getRandomPosition(Vector pos) {
	// Generate random numbers from -4 to 5
	double x = (double)(rand() % 800) / 100 - 4;
	double y = (double)(rand() % 800) / 100 - 4;
	//int z = (double)(rand() % 800) / 100 - 4;
	int z = 0;
	setVector(pos, x, y, z);
}

void BoidSystem::generateInitalBoids(double numBoids) {
	for (int i = 0; i < flocks.size(); i++) {
		flocks[i].members.clear();
	}
	
	for (int i = 0; i < numBoids; i++) {
		Vector initalPosition, initalDirection;
		getRandomPosition(initalPosition);
		getRandomDirection(initalDirection);
		Boid* newBoid = new Boid(initalPosition, initalDirection, i, i % FLOCK_COUNT);
		flocks[newBoid->flockId].members.push_back(newBoid);
	}
	glutPostRedisplay();
};

void BoidSystem::getState(double* p) {
	BoidState* state = (BoidState*) p;
	state->flocks = &flocks;
};

void BoidSystem::setState(double* p) {
	
}

void BoidSystem::reset(double time) {
	for (int i = 0; i < flocks.size(); i++) {
		// for (int j = 0; j < flocks[i].members.size(); j++) {
		for (std::list<Boid*>::iterator iter = flocks[i].members.begin(); iter != flocks[i].members.end(); ++i) {
			VecCopy((*iter)->position, (*iter)->initalPosition);
		}
	}
}

void BoidSystem::display(GLenum mode) {
	glPointSize(3);
	set_colour(0.0, 1.0, 0.0);
	Vector tail, tailOffset, leftWing, rightWing, wingOffset, zAxis;
	setVector(zAxis, 0, 0, 1);
	
	
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
			glBegin(GL_TRIANGLE_FAN);
			// Calculate vector from head to tail
			VecCopy(tailOffset, b->velocity);
			VecScale(tailOffset, -0.3);
			// Calculate vector orthoginal to that vector
			VecCrossProd(wingOffset, zAxis, tailOffset);
			VecNormalize(wingOffset);
			VecScale(wingOffset, 0.15);


			// Calculate tail position
			VecAdd(tail, b->position, tailOffset);
			// Calculate right wing position
			VecAdd(rightWing, tail, wingOffset);
			VecScale(tailOffset, 0.4);
			VecAdd(rightWing, tailOffset, rightWing);

			// Calculate left wing position
			VecSubtract(leftWing, tail, wingOffset);
			VecAdd(leftWing, tailOffset, leftWing);
			// Draw the boids
			glVertex3dv(b->position);
			glVertex3dv(leftWing);
			glVertex3dv(tail);
			glVertex3dv(rightWing);
			glEnd();
		}
	}
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