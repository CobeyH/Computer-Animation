#pragma once
#include "BaseSystem.h"
#include "BodyPart.h"
#include <vector>

#define UPPER_ARM_RATIO 1.5
#define LOWER_ARM_RATIO 1.5
#define TORSO_RATIO 2.5
#define LEG_RATIO 1.3
#define HAND_RATIO 0.5
#define HEAD_RATIO 0.75

enum Orientation {
	Left,
	Right
};

class HumanModel : public BaseSystem {
public:
	HumanModel(const std::string& name);
	void getState(double* p);
	void setState(double* p);
	void reset(double time);
	void display(GLenum mode = GL_RENDER);
	int command(int argc, myCONST_SPEC char** argv);
private:
	void createBody();
	void createArm(Orientation side);
	void createLeg(Orientation side);
	BodyPart* root;
	double* angles[7];
};