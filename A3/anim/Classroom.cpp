#include "Classroom.h"

Classroom::Classroom(const std::string& name) : BaseSystem(name) {

};

void Classroom::getState(double* p) {

};

void Classroom::setState(double* p) {

};

void Classroom::reset(double time) {

};

void Classroom::display(GLenum mode) {

	glPushMatrix();

	glDisable(GL_LIGHTING);
	// Back wall
	glBegin(GL_QUADS);
	glVertex3d(ROOM_WIDTH, ROOM_HEIGHT, -0.02);
	glVertex3d(ROOM_WIDTH, -ROOM_HEIGHT, -0.02);
	glVertex3d(-ROOM_WIDTH, -ROOM_HEIGHT, -0.02);
	glVertex3d(-ROOM_WIDTH, ROOM_HEIGHT, -0.02);
	glEnd();
	glEnable(GL_LIGHTING);
	// Blackboard
	glBegin(GL_QUADS);
	glVertex3d(BOARD_WIDTH, BOARD_HEIGHT, -0.01);
	glVertex3d(BOARD_WIDTH, -BOARD_HEIGHT, -0.01);
	glVertex3d(-BOARD_WIDTH, -BOARD_HEIGHT, -0.01);
	glVertex3d(-BOARD_WIDTH, BOARD_HEIGHT, -0.01);
	glEnd();
	//Floor
	glTranslated(0, -ROOM_HEIGHT, ROOM_DEPTH);
	glBegin(GL_QUADS);
	glVertex3d(ROOM_WIDTH, 0, ROOM_DEPTH);
	glVertex3d(ROOM_WIDTH, 0, -ROOM_DEPTH);
	glVertex3d(-ROOM_WIDTH, 0, -ROOM_DEPTH);
	glVertex3d(-ROOM_WIDTH, 0, ROOM_DEPTH);
	glEnd();
	glPopMatrix();
};

int Classroom::command(int argc, myCONST_SPEC char** argv) {
	return TCL_OK;
};