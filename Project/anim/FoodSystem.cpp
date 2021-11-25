#include "FoodSystem.h"

FoodSystem::FoodSystem(const std::string& name) : BaseSystem(name) {
	generateFood();
};

void FoodSystem::generateFood() {
	Vector position;
	for (int i = 0; i < FOOD_LIMIT; i++) {
		double x = (double)(rand() % 1200) / 100 - 6;
		double y = (double)(rand() % 1200) / 100 - 6;
		Food* newFood = new Food;
		setVector(newFood->position, x, y, 0);
		foodLocations[i] = newFood;
	}
}

void FoodSystem::getState(double* p) {
	GetFoodState* state = (GetFoodState*) p;
	state->food = foodLocations;
	state->foodQuantity = FOOD_LIMIT;
	p = (double*) &state;
}

void FoodSystem::setState(double* p) {

}

void FoodSystem::reset(double time) {

}

void FoodSystem::display(GLenum mode) {
	glBegin(GL_POINTS);
	for (int i = 0; i < FOOD_LIMIT; i++) {
		glVertex3dv(foodLocations[i]->position);
	}
	glEnd();
}

int FoodSystem::command(int argc, myCONST_SPEC char** argv) {
	return TCL_ERROR;
}