#pragma once

struct ParticleLock {
	int index;
	bool shouldLock;
};

struct Particle {
	Particle(Vector initalPos, Vector initalVel, double particleMass) {
		VecCopy(position, initalPos);
		VecCopy(velocity, initalVel);
		mass = particleMass;
		locked = false;
	}
	Vector position;
	Vector velocity;
	double mass;
	bool locked;
};

struct Spring {
	Spring(int start, int end, double sConstant, double sDisplacement, double length) {
		startPoint = start;
		endPoint = end;
		ks = sConstant;
		kd = sDisplacement;
		restLength = length;
	}
	int startPoint;
	int endPoint;
	double ks;
	double kd;
	double restLength;
};
