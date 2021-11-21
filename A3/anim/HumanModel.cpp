#include "HumanModel.h"
#include "BodyPart.h"
#include "States.h"

HumanModel::HumanModel(const std::string& name) : BaseSystem(name) {
	root = new BodyPart("root", TORSO_RATIO / 2.5 * 2, TORSO_RATIO * 2, 0);
	setVector(root->offset, 0, 0, 7);
	createBody();
	// Set up angles for shoulder
	BodyPart* rightShoulder = root->getChild(1);
	angles[0] = rightShoulder->linkXRotation();
	angles[1] = rightShoulder->linkYRotation();
	angles[2] = rightShoulder->linkZRotation();
	
	BodyPart* rightElbow = rightShoulder->getChild(0);
	angles[3] = rightElbow->linkXRotation();
	angles[4] = rightElbow->linkYRotation();

	BodyPart* rightWrist = rightElbow->getChild(0);
	angles[5] = rightWrist->linkZRotation();
	angles[6] = rightWrist->linkYRotation();

	setStartingAngles();

};

void HumanModel::setStartingAngles() {
	*angles[2] = -90 * DEG_TO_RAD;
	BodyPart* leftShoulder = root->getChild(2);
	leftShoulder->rotZ = 90 * DEG_TO_RAD;
}

void HumanModel::setRestingAngles() {
	// Set inital angles
	*angles[0] = 10 * DEG_TO_RAD;
	*angles[1] = 35 * DEG_TO_RAD;
	*angles[2] = -160 * DEG_TO_RAD;
	*angles[3] = -80 * DEG_TO_RAD;
	for (int i = 4; i < 7; i++) {
		*angles[i] = 5 * DEG_TO_RAD;
	}
	recalculateEffectorPos();

	BodyPart* leftShoulder = root->getChild(2);
	leftShoulder->rotZ = 160 * DEG_TO_RAD;
	BodyPart* leftElbow = leftShoulder->getChild(0);
	leftElbow->rotZ = 10 * DEG_TO_RAD;
	BodyPart* leftWrist = leftElbow->getChild(0);
	leftWrist->rotZ = 10 * DEG_TO_RAD;

	glutPostRedisplay();
}

void HumanModel::createArm(Orientation side) {
	int ts = side == Left ? -1 : 1;
	BodyPart* upperArm = new BodyPart("UpperArm", UPPER_ARM_RATIO / 2, UPPER_ARM_RATIO * 2, 1);
	BodyPart* lowerArm = new BodyPart("LowerArm", LOWER_ARM_RATIO / 2, LOWER_ARM_RATIO * 2, 2);
	BodyPart* hand = new BodyPart("Hand", HAND_RATIO, HAND_RATIO * 2, 3);

	setVector(upperArm->offset, ts * TORSO_RATIO / 3, UPPER_ARM_RATIO, 0);
	setVector(lowerArm->offset, 0, UPPER_ARM_RATIO * 2, 0);
	setVector(hand->offset, 0, LOWER_ARM_RATIO * 2, 0);
	

	lowerArm->addChild(hand);
	upperArm->addChild(lowerArm);
	root->addChild(upperArm);
}

void HumanModel::createLeg(Orientation side) {
	int ts = side == Left ? -1 : 1;
	BodyPart* upperLeg = new BodyPart("upperLeg", LEG_RATIO / 2, LEG_RATIO * 2, 1);
	BodyPart* lowerLeg = new BodyPart("lowerLeg", LEG_RATIO / 2, LEG_RATIO * 2, 2);
	BodyPart* foot = new BodyPart("foot", HAND_RATIO * 2, HAND_RATIO, 3);

	setVector(upperLeg->offset, ts * TORSO_RATIO/4, - TORSO_RATIO * 0.9, 0);
	upperLeg->rotZ = 180.0 * PI / 180.0;

	lowerLeg->addChild(foot);
	upperLeg->addChild(lowerLeg);
	root->addChild(upperLeg);
}

void HumanModel::createBody() {
	BodyPart* head = new BodyPart("head", HEAD_RATIO * 2, HEAD_RATIO * 2, 1);
	setVector(head->offset, 0, TORSO_RATIO, 0);
	root->addChild(head);

	createArm(Right);
	createArm(Left);
	
	createLeg(Left);
	createLeg(Right);
}

void HumanModel::getState(double* p) {
	VecCopy(p, effectorPos);
};

void HumanModel::setState(double* p) {
	SetBobState* update = (SetBobState*)p;
	switch (update->mode) {
	case newTarget:
	{
		Eigen::Vector<double, 3> pTarget;
		pTarget[0] = update->target[0];
		pTarget[1] = update->target[1];
		pTarget[2] = update->target[2];
		VecCopy(targetPoint, update->target);
		computeJacobian(pTarget);
	}
		break;
	case startup:
		setRestingAngles();
		recalculateEffectorPos();
		break;
	}
	
};

void HumanModel::reset(double time) {
	setRestingAngles();
	glutPostRedisplay();
};

void HumanModel::display(GLenum mode) {
	glPointSize(10);
	glBegin(GL_POINTS);
	set_colour(1, 0, 0);
	glVertex3f(effectorPos[0], effectorPos[1], effectorPos[2]);
	set_colour(0, 1, 0);
	glVertex3f(targetPoint[0], targetPoint[1], targetPoint[2]);
	glEnd();

	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	GLfloat params[] = { 3.0, 3.0, 3.0, 1.0 };
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, params);
	set_colour(1, 1, 0.1);
	glTranslated(root->offset[0], root->offset[1], root->offset[2]);

	root->drawRoot();
	
	set_colour(1, 1, 1);
	glPopAttrib();
	glPopMatrix();

};

void HumanModel::recalculateEffectorPos() {
	Jacobian* j = new Jacobian(this);
	Eigen::Vector<double, 3> pCurr = j->computeColumn(0);
	effectorPos[0] = pCurr[0]; effectorPos[1] = pCurr[1]; effectorPos[2] = pCurr[2];
}

void HumanModel::computeJacobian(Eigen::Vector<double, 3> pTarget) {
	Jacobian* jacobian = new Jacobian(this);
	jacobian->computeJacobian();

	Eigen::Vector<double, 3> pCurr = {effectorPos[0], effectorPos[1], effectorPos[2]};

	Eigen::Vector<double, 3> error = pTarget - pCurr;
	Eigen::Matrix<double, 7, 3> Jt = jacobian->jacobian.transpose();
	Eigen::Matrix<double, 3, 3> JJt = jacobian->jacobian * Jt;	
	//Eigen::JacobiSVD<Eigen::MatrixXd> svd(JJt, Eigen::ComputeThinU | Eigen::ComputeThinV);
	//Eigen::Vector<double, 3> beta = svd.solve(error);
	Eigen::PartialPivLU<Eigen::MatrixXd> lu(JJt);
	Eigen::Vector<double, 3> beta = lu.solve(error);
	Eigen::Vector<double, 7> angleChange = Jt * beta;
	//Eigen::Vector<double, 7> angleChange = Jt * error;
	for (int i = 0; i < 7; i++) {

		*angles[i] += angleChange(i);
		if (angleChange(i) > 0.05 || angleChange(i) < -0.05) {
			double errorSize = error.norm();
			double x = 1;
		}
	}
	// Update the rotation matricies angles
	recalculateEffectorPos();
	animTcl::OutputMessage("Effector Pos %f %f %f", effectorPos[0], effectorPos[1], effectorPos[2]);
	glutPostRedisplay();
}

int HumanModel::command(int argc, myCONST_SPEC char** argv) {
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}

	else if (strcmp(argv[0], "position") == 0) {
		if (argc != 4) {
			animTcl::OutputMessage("Invalid arguments passed. Expeced 4 arguments but got %d", argc);
			return TCL_ERROR;
		}
		setVector(root->offset, atof(argv[1]), atof(argv[2]), atof(argv[3]));
		recalculateEffectorPos();
		glutPostRedisplay();
	}
	else if (strcmp(argv[0], "setTheta") == 0) {
		if (argc != 3) {
			animTcl::OutputMessage("Invalid arguments passed. Expeced 3 arguments but got %d", argc);
			return TCL_ERROR;
		}
		int index = atoi(argv[1]);
		if (index > 6 || index < 1) {
			animTcl::OutputMessage("Invalid theta value. Please specify a value from 1-7");
			return TCL_ERROR;
		}
		*angles[index - 1] = atof(argv[2]);
		glutPostRedisplay();
	}
	else {
		animTcl::OutputMessage("The given command %s is not valid", argv[0]);
	}
};