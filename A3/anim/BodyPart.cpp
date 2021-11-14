#include "BodyPart.h"
#include "BodyPart.h"
#include <vector>
#include <Eigen/Dense>

void BodyPart::drawEllipse(double rx, double ry)
{
    float theta = 2 * PI / float(NUM_SEGMENTS);
    float c = cosf(theta); //precalculate the sine and cosine
    float s = sinf(theta);
    float t;

    float x = 1; //we start at angle = 0
    float y = 0;

    glBegin(GL_LINE_LOOP);
    for (int ii = 0; ii < NUM_SEGMENTS; ii++)
    {
        //apply radius and offset
        glVertex2f(x * rx, y * ry);//output vertex 

        //apply the rotation
        t = x;
        x = c * x - s * y;
        y = s * t + c * y;
    }
    glEnd();
}

BodyPart::BodyPart(const std::string& name, double bodyWidth, double bodyHeight, int depth) : BaseSystem(name) {
    width = bodyWidth;
    height = bodyHeight;
    limbDepth = depth;
    zeroVector(offset);
    rotX = NULL; rotY = NULL; rotZ = NULL;
    children.clear();
};

void BodyPart::addChild(BodyPart* child) {
    children.push_back(*child);
}

BodyPart* BodyPart::getChild(int index) {
    return &children[index];
}

double* BodyPart::linkXRotation(void) {
    return &rotX;
}

double* BodyPart::linkYRotation(void) {
    return &rotY;
}

double* BodyPart::linkZRotation(void) {
    return &rotZ;
}

void BodyPart::setPosition(Vector pos) {
    VecCopy(offset, pos);
}

void BodyPart::getState(double* p) {

};

void BodyPart::setState(double* p) {

};

void BodyPart::reset(double time) {

};

void BodyPart::drawRoot() {
    drawEllipse(width, height);
    for (BodyPart p : children) {
        glPushMatrix();
        glTranslated(p.offset[0], p.offset[1], p.offset[2]);
        glRotated(p.rotX, 1, 0, 0);
        glRotated(p.rotY, 0, 1, 0);
        glRotated(p.rotZ, 0, 0, 1);
        p.display();
        glPopMatrix();
    }
}

// Methods for getting rotation matricies
Eigen::Matrix<double, 4, 4> BodyPart::getXRotMatrix() {
    Eigen::Matrix<double, 4, 4> xRotationMatrix{
      {1, 0, 0, 0},
      {0, cos(rotX), -sin(rotX), 0},
      {0, sin(rotX), cos(rotX), 0},
      {0, 0, 0, 1}
    };
    return xRotationMatrix;
}

Eigen::Matrix<double, 4, 4> BodyPart::getYRotMatrix() {
    Eigen::Matrix<double, 4, 4> rotationMatrix{
      {cos(rotY), 0, sin(rotY), 0},
      {0, 1, 0, 0},
      {-sin(rotY), 0, cos(rotY), 0},
      {0, 0, 0, 1}
    };
    return rotationMatrix;
}

Eigen::Matrix<double, 4, 4> BodyPart::getZRotMatrix() {
  Eigen::Matrix<double, 4, 4> rotationMatrix{
      {cos(rotZ), -sin(rotZ), 0, 0},
      {sin(rotZ), cos(rotZ), 0, 0},
      {0, 0, 1, 0},
      {0, 0, 0, 1}
    };
    return rotationMatrix;
}

// Methods for getting derivatives of rotation matricies
Eigen::Matrix<double, 4, 4> BodyPart::getXRotMatrixDer() {
    Eigen::Matrix<double, 4, 4> rotationMatrix{
       {1, 0, 0, 0},
       {0, -sin(rotX), -cos(rotX), 0},
       {0, cos(rotX), -sin(rotX), 0},
       {0, 0, 0, 1}
    };
    return rotationMatrix;
}

Eigen::Matrix<double, 4, 4> BodyPart::getYRotMatrixDer() {
    Eigen::Matrix<double, 4, 4> rotationMatrix{
       {-sin(rotY), 0, cos(rotY), 0},
       {0, 1, 0, 0},
       {-cos(rotY), 0, -sin(rotY), 0},
       {0, 0, 0, 1}
    };
    return rotationMatrix;
}

Eigen::Matrix<double, 4, 4> BodyPart::getZRotMatrixDer() {
    Eigen::Matrix<double, 4, 4> rotationMatrix{
     {-sin(rotZ), -cos(rotZ), 0, 0},
     {cos(rotZ), -sin(rotZ), 0, 0},
     {0, 0, 1, 0},
     {0, 0, 0, 1}
    };
    return rotationMatrix;
}

void BodyPart::display(GLenum mode) {
	glMatrixMode(GL_MODELVIEW);
    glPushMatrix();    
    glTranslated(0, height, 0);
    drawEllipse(width, height);
    for (BodyPart p : children) {
        glPushMatrix();
        // TODO: Fix code duplication
        glTranslated(0, height, 0);
        glRotated(p.rotX, 1, 0, 0);
        glRotated(p.rotY, 0, 1, 0);
        glRotated(p.rotZ, 0, 0, 1);
        p.display();
        glPopMatrix();
    }
  
	glPopMatrix();
};

int BodyPart::command(int argc, myCONST_SPEC char** argv) {
	return TCL_OK;
};