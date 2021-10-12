#pragma once
#include <vector>

struct Particle {
	Particle(Vector initalPos, Vector initalVel, double particleMass) {
		VecCopy(position, initalPos);
		VecCopy(velocity, initalVel);
		mass = particleMass;
		locked = false;
	}
	Vector position;
	Vector velocity;
	Vector forces;
	double mass;
	bool locked;
};

struct ParticleState {
	std::vector<Particle>* particles;
};

struct ParticleLock {
	int index;
	bool shouldLock;
};

struct Spring {
	Spring(int start, int end, double sConstant, double kDrag, double length) {
		startPoint = start;
		endPoint = end;
		ks = sConstant;
		kd = kDrag;
		restLength = length;
	}
	int startPoint;
	int endPoint;
	double ks;
	double kd;
	double restLength;
};
