#pragma once
#include <vector>
#include <list>

struct Attributes {
	double mass, cohesion, separation, align, maxSpeed;
};

struct Boid {
	Boid(Vector initalPos, Vector initalDir, int index, int flock, double speed, bool predator = false) {
		VecCopy(initalPosition, initalPos);
		VecCopy(position, initalPos);
		VecCopy(velocity, initalDir);
		attrib.maxSpeed = speed;
		id = index;
		flockId = flock;
		isPredator = predator;
		hunger = 0;
	}
	Vector initalPosition;
	Vector position;
	Vector velocity;
	int id, flockId, hunger;
	bool isPredator;
	Attributes attrib;
};

struct Food {
	Vector position;
};

struct Flock {
	std::list<Boid*> members;
};

struct BoidState {
	std::vector<Flock>* flocks;
	Flock* predators;
};

struct BoidSetState {
	Boid* toDelete;
};

struct GetFoodState {
	Food** food;
	int foodQuantity;
};