#pragma once
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "States.h"
#include <vector>
#include "QuadTree.h"

#define SCREEN_EDGE 6
#define EDGE_MARGIN 0.2

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
	std::vector<QuadTree*> quadTrees;
	void calculateFlockCenter(Vector center, Flock* flock);
	void updateDirection(Boid* b, Vector center, Flock* flock);
	void addCohesion(Boid* b, Vector center, Vector desiredVelocity);
	void addAlignment(Boid* b, Flock* flock, Vector desiredVelocity);
	void addSeparation(Boid* b, Flock* flock, Vector desiredVelocity);
	void avoidPredators(Flock* normalBirds, Flock* predators, QuadTree* qTree);
	void checkPredatorFood(Boid* p, Flock* flock);
};

