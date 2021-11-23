#pragma once
#include <vector>
#include <list>

struct Boid {
	Boid(Vector initalPos, Vector initalDir, int index, int flock, bool predator = false) {
		VecCopy(initalPosition, initalPos);
		VecCopy(position, initalPos);
		VecCopy(velocity, initalDir);
		speed = 0.01;
		id = index;
		flockId = flock;
		isPredator = predator;
	}
	double speed, mass;
	Vector initalPosition;
	Vector position;
	Vector velocity;
	int id, flockId;
	bool isPredator;
};

struct Flock {
	std::list<Boid*> members;
};

struct BoidState {
	std::vector<Flock>* flocks;
	Flock* predators;
};

