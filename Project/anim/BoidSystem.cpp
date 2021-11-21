#include "BoidSystem.h"
#include "BaseSystem.h"
#include <vector>

BoidSystem::BoidSystem(const std::string& name) : BaseSystem(name) {

};

void getRandomDirection(Vector dir) {
	int x = rand() % 100 - 50;
	int y = rand() % 100 - 50;
	int z = rand() % 100 - 50;
	setVector(dir, x, y, z);
	VecNormalize(dir);
}
void getRandomPosition(Vector pos) {
	// Generate random numbers from -4 to 5
	int x = (double)(rand() % 800) / 100 - 4;
	int y = (double)(rand() % 800) / 100 - 4;
	int z = (double)(rand() % 800) / 100 - 4;
	setVector(pos, x, y, z);
}

void BoidSystem::generateInitalBoids(double numBoids) {
	boids.clear();
	for (int i = 0; i < numBoids; i++) {
		Vector initalPosition, initalDirection;
		getRandomPosition(initalPosition);
		getRandomDirection(initalDirection);
		Boid* newBoid = new Boid(initalPosition, initalDirection, i);
		boids.push_back(*newBoid);
	}
	glutPostRedisplay();
};

void BoidSystem::getState(double* p) {
	BoidState* state = (BoidState*) p;
	state->boids = &boids;
};

void BoidSystem::setState(double* p) {
	
}

void BoidSystem::reset(double time) {
	for (int i = 0; i < boids.size(); i++) {
		VecCopy(boids[i].position, boids[i].initalPosition);
	}
}

void BoidSystem::display(GLenum mode) {
	glPointSize(3);
	set_colour(0.0, 1.0, 0.0);
	Vector tail, tailOffset, leftWing, rightWing, wingOffset, zAxis;
	setVector(zAxis, 0, 0, 1);
	
	
	for (Boid b : boids) {
		glBegin(GL_TRIANGLE_FAN);
		// Calculate vector from head to tail
		VecCopy(tailOffset, b.velocity);
		VecScale(tailOffset, -0.5);
		// Calculate vector orthoginal to that vector
		VecCrossProd(wingOffset, zAxis, tailOffset);
		VecNormalize(wingOffset);
		VecScale(wingOffset, 0.15);

		
		// Calculate tail position
		VecAdd(tail, b.position, tailOffset);
		// Calculate right wing position
		VecAdd(rightWing, tail, wingOffset);
		VecScale(tailOffset, 0.4);
		VecAdd(rightWing, tailOffset, rightWing);
		
		// Calculate left wing position
		VecSubtract(leftWing, tail, wingOffset);
		VecAdd(leftWing, tailOffset, leftWing);
		// Draw the boids
		glVertex3dv(b.position);
		glVertex3dv(leftWing);
		glVertex3dv(tail);
		glVertex3dv(rightWing);
		glEnd();
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