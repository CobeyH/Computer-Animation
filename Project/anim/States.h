#pragma once
#include <vector>
#include <list>

struct Boid {
	Boid(Vector initalPos, Vector initalDir, int index, int flock, double speed, bool predator = false) {
		VecCopy(initalPosition, initalPos);
		VecCopy(position, initalPos);
		VecCopy(velocity, initalDir);
		maxSpeed = speed;
		id = index;
		flockId = flock;
		isPredator = predator;
	}
	double maxSpeed, mass;
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

