#include "ParticleSystem.h"

ParticleSystem::ParticleSystem(const std::string& name) : BaseSystem(name) {

};

void ParticleSystem::generateInitalParticles(double numParticles) {
	for (int i = 0; i < numParticles; i++) {
		Vector initalValues;
		zeroVector(initalValues);
		Particle* newParticle = new Particle(initalValues, initalValues, 0);
		particles.push_back(*newParticle);
	}
}

void ParticleSystem::updateParticle(int index, Vector pos, Vector vel, double mass) {
	Particle* p = &particles[index];
	VecCopy(p->position, pos);
	VecCopy(p->velocity, vel);
	p->mass = mass;
	glutPostRedisplay();
};

void ParticleSystem::updateAllVelocities(Vector newVelocity) {
	for (Particle p : particles) {
		VecCopy(p.velocity, newVelocity);
	}
}

void ParticleSystem::getState(double* p) {
};

void ParticleSystem::setState(double* p) {
};

void ParticleSystem::reset(double time) {
};

void ParticleSystem::display(GLenum mode) {
	glPointSize(3);
	glBegin(GL_POINTS);
	glColor3f(1.0, 0, 0);
	for (Particle p : particles) {
		glVertex3dv(p.position);
	}
	glEnd();
};

int ParticleSystem::command(int argc, myCONST_SPEC char** argv) {
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "dim") == 0) {
		generateInitalParticles(atof(argv[1]));
	}
	else if (strcmp(argv[0], "particle") == 0) {
		int index = atoi(argv[1]);
		double mass = atof(argv[2]);
		double posX = atof(argv[3]);
		double posY = atof(argv[4]);
		double posZ = atof(argv[5]);
		double velX = atof(argv[6]);
		double velY = atof(argv[7]);
		double velZ = atof(argv[8]);

		Vector position, velocity;
		setVector(position, posX, posY, posZ);
		setVector(velocity, velX, velY, velZ);
		updateParticle(index, position, velocity, mass);
	}
	else if (strcmp(argv[0], "all_velocities") == 0) {
		double velX = atof(argv[1]);
		double velY = atof(argv[2]);
		double velZ = atof(argv[3]);
		Vector velocity;
		updateAllVelocities(velocity);
	}
};