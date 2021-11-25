#pragma once
#include "BaseSystem.h"
#include "States.h"

#define FOOD_LIMIT 50

class FoodSystem : public BaseSystem {
public:
	// Inherited functions
	FoodSystem(const std::string& name);
	void getState(double* p);
	void setState(double* p);
	void reset(double time);
	void display(GLenum mode = GL_RENDER);
	int command(int argc, myCONST_SPEC char** argv);
private:
	Food* foodLocations[FOOD_LIMIT];
	void generateFood();
};

