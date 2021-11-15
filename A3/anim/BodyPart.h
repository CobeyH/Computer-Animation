#pragma once
#include "BaseSystem.h"
#include <vector>
#include <Eigen/Dense>

#define NUM_SEGMENTS 50

class BodyPart : public BaseSystem {
public:
	BodyPart(const std::string& name, double bodyWidth, double bodyHeight, int depth);
	void getState(double* p);
	void setState(double* p);
	void reset(double time);
	void display(GLenum mode = GL_RENDER);
	int command(int argc, myCONST_SPEC char** argv);
	void drawEllipse(double rx, double ry);
	void addChild(BodyPart* child);
	BodyPart* getChild(int index);
	void setPosition(Vector pos);
	void drawRoot();
	double getHeight() { return height; };
	Vector offset;
	// Methods for linking body part to body
	double* linkXRotation(void);
	double* linkYRotation(void);
	double* linkZRotation(void);
	// Methods for getting rotation matricies
	Eigen::Matrix<double, 4, 4> getXRotMatrix(void);
	Eigen::Matrix<double, 4, 4> getYRotMatrix();
	Eigen::Matrix<double, 4, 4> getZRotMatrix();
	// Methods for getting derivatives of rotation matricies
	Eigen::Matrix<double, 4, 4> getXRotMatrixDer();
	Eigen::Matrix<double, 4, 4> getYRotMatrixDer();
	Eigen::Matrix<double, 4, 4> getZRotMatrixDer();
	Eigen::Matrix<double, 4, 4> getTranslationMatrix();

	double rotX, rotY, rotZ;
private:
	double width, height;
	int limbDepth;
	std::vector<BodyPart> children;
};