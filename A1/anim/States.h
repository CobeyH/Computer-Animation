#pragma once
#include <vector>
struct Particle;
struct Spring;

struct Particle {
	Particle(Vector initalPos, Vector initalVel, double particleMass) {
		VecCopy(position, initalPos);
		VecCopy(prevPos, initalPos);
		VecCopy(velocity, initalVel);
		VecCopy(initalVelocity, initalVel);
		zeroVector(externalSpringForce);
		mass = particleMass;
		locked = false;
	}
	Vector position;
	Vector initalPosition;
	Vector prevPos;
	Vector velocity;
	Vector initalVelocity;
	double mass;
	bool locked;
	std::vector<Spring> connectedSprings;
	Vector externalSpringForce;
};

struct Spring {
	Spring(Particle* terminalPoint, double sConstant, double kDrag, double length) {
		endPoint = terminalPoint;
		ks = sConstant;
		kd = kDrag;
		restLength = length;
	}
	Particle* endPoint;
	Particle* p2;
	double ks;
	double kd;
	double restLength;
};

enum UpdateType {
	Lock,
	NewSpring
};

struct UpdateState {
	UpdateState(UpdateType operation) {
		mode = operation;
		index = NULL;
		shouldLock = NULL;
		spring = NULL;
	};
	UpdateType mode;
	int index;
	bool shouldLock;
	Spring* spring;
};

struct ParticleState {
	std::vector<Particle>* particles;
};