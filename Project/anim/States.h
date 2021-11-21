#pragma once
#include <vector>

struct Boid {
	Boid(Vector initalPos, Vector initalDir, int index) {
		VecCopy(initalPosition, initalPos);
		VecCopy(position, initalPos);
		VecCopy(velocity, initalDir);
		speed = 0.01;
		id = index;
	}
	double speed, mass;
	Vector initalPosition;
	Vector position;
	Vector velocity;
	int id;
};

struct BoidState {
	std::vector<Boid>* boids;
};

