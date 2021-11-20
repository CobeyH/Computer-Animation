#pragma once
#include "BaseSystem.h"
#include <vector>

#define ROOM_WIDTH 12
#define ROOM_HEIGHT 15
#define ROOM_DEPTH 20

#define BOARD_WIDTH 8
#define BOARD_HEIGHT 6

class Classroom : public BaseSystem {
public:
	Classroom(const std::string& name);
	void getState(double* p);
	void setState(double* p);
	void reset(double time);
	void display(GLenum mode = GL_RENDER);
	int command(int argc, myCONST_SPEC char** argv);
};