#pragma once
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "States.h"
#include "QuadTree.h"
#include "FoodSystem.h"
#include <vector>
#include <thread>
#include <iostream>
#include <fstream>

#define PREDATOR_STARVATION_THREASHOLD 0.25
#define BOID_STARVATION_THREASHOLD 0.25
#define STARVATION 3000

#define PREDATOR_INFLUENCE_RANGE 0.5
#define BOID_PERCEPTION_RANGE 3
#define BOID_EATING_DISTANCE 0.5

#define SCREEN_WIDTH 12
#define SCREEN_EDGE SCREEN_WIDTH / 2
#define EDGE_MARGIN 0.2
#define WALL_REPULSION 0.05

#define PREDATOR_AVOIDANCE_STRENGTH 0.5
#define TURNING_RATE 0.05

#define COHESION_STRENGTH 0.02
#define ALIGNMENT_STRENGTH 0.03
#define SEPARATION_STRENGTH 0.02
#define FOOD_ATTRACTION_STRENGTH 0.05

#define PRINT_INTERVAL 30



class BoidSimulator : public BaseSimulator {
public:
	BoidSimulator(const std::string& name, BaseSystem* target);
	int step(double time);
	int init(double time) {
		double prevTime = 0;
		prevPrint = 0;
		return 0;
	};
	void display(GLenum mode = GL_RENDER);
	void registerFoodSystem(BaseSystem* system);
protected:
	double prevTime, prevPrint;
	BaseSystem* m_object;
	std::list<Boid*> starvedBoids;
	BaseSystem* foodSystem;
	QuadTree<Food>* foodQTree;
	void printInfo(BoidState* state, double time);
	void updateAllBoids(BoidState* state, double deltaTime);
	void updateFlockMembers(Flock* flock, Flock* predators, double deltaTime);
	void updatePosition(Boid* b, double deltaTime);
	void calculateFlockCenter(Vector center, Flock* flock);
	void updateDirection(Boid* b, Vector center, Boid* closeBoids[], int size);
	void addCohesion(Boid* b, Vector center, Vector desiredVelocity);
	void addAlignment(Boid* b, Boid** closeBoids, int size, Vector desiredVelocity);
	void addSeparation(Boid* b, Boid** closeBoids, int size, Vector desiredVelocity);
	void addFoodAttraction(Boid* b, Vector desiredVelocity);
	void avoidPredators(Flock* normalBirds, Flock* predators, QuadTree<Boid>* qTree);
	void checkPredatorFood(Boid* p, Boid* closeBoids[], int flockSize);
	void killBoid(Boid* b);
};

