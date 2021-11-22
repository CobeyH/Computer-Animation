#pragma once
#include <vector>

struct Boid {
	Boid(Vector initalPos, Vector initalDir, int index, int flock) {
		VecCopy(initalPosition, initalPos);
		VecCopy(position, initalPos);
		VecCopy(velocity, initalDir);
		speed = 0.01;
		id = index;
		flockId = flock;
	}
	double speed, mass;
	Vector initalPosition;
	Vector position;
	Vector velocity;
	int id, flockId;
};

struct Flock {
	std::vector<Boid> members;
};

struct BoidState {
	std::vector<Flock>* flocks;
};

