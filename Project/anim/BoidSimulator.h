#pragma once
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "States.h"
#include <vector>
#include "QuadTree.h"
#include <thread>

#define SCREEN_EDGE 6
#define EDGE_MARGIN 0.2

#define STARVATION 1000

class BoidSimulator : public BaseSimulator {
public:
	BoidSimulator(const std::string& name, BaseSystem* target);
	int step(double time);
	int init(double time) {
		double prevTime = 0;
		return 0;
	};
	void display(GLenum mode = GL_RENDER);
protected:
	double prevTime;
	BaseSystem* m_object;
	std::list<Boid*> starvedBoids;
	void updateAllBoids(BoidState* state, double deltaTime);
	void updateFlockMembers(Flock* flock, Flock* predators, double deltaTime);
	void updatePosition(Boid* b, double deltaTime);
	void calculateFlockCenter(Vector center, Flock* flock);
	void updateDirection(Boid* b, Vector center, Boid* closeBoids[], int size);
	void addCohesion(Boid* b, Vector center, Vector desiredVelocity);
	void addAlignment(Boid* b, Boid** closeBoids, int size, Vector desiredVelocity);
	void addSeparation(Boid* b, Boid** closeBoids, int size, Vector desiredVelocity);
	void avoidPredators(Flock* normalBirds, Flock* predators, QuadTree* qTree);
	void checkPredatorFood(Boid* p, Boid* closeBoids[], int flockSize);
	void killBoid(Boid* b);
};

