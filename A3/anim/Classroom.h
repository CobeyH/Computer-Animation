#pragma once
#include "BaseSystem.h"
#include <vector>

#define ROOM_WIDTH 8
#define ROOM_HEIGHT 8
#define ROOM_DEPTH 10

#define BOARD_WIDTH 6
#define BOARD_HEIGHT  5

class Classroom : public BaseSystem {
public:
	Classroom(const std::string& name);
	void getState(double* p);
	void setState(double* p);
	void reset(double time);
	void display(GLenum mode = GL_RENDER);
	int command(int argc, myCONST_SPEC char** argv);
};