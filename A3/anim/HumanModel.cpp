#include "HumanModel.h"
#include "BodyPart.h"

HumanModel::HumanModel(const std::string& name) : BaseSystem(name) {
	root = new BodyPart("root", TORSO_RATIO / 2.5, TORSO_RATIO, 0);
	setVector(root->offset, 0, 0, 2);
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
	angles[5] = rightElbow->linkZRotation();
	angles[6] = rightElbow->linkYRotation();
};

void HumanModel::createArm(Orientation side) {
	int ts = side == Left ? -1 : 1;
	BodyPart* upperArm = new BodyPart("UpperArm", UPPER_ARM_RATIO / 4, UPPER_ARM_RATIO, 1);
	BodyPart* lowerArm = new BodyPart("LowerArm", LOWER_ARM_RATIO / 4, LOWER_ARM_RATIO, 2);
	BodyPart* hand = new BodyPart("Hand", HAND_RATIO / 2, HAND_RATIO, 3);

	setVector(upperArm->offset, ts * TORSO_RATIO / 3, UPPER_ARM_RATIO, 0);
	upperArm->rotZ = -ts * 90;
	lowerArm->rotZ = 80;
	hand->rotZ = 80;
	

	lowerArm->addChild(hand);
	upperArm->addChild(lowerArm);
	root->addChild(upperArm);
}

void HumanModel::createLeg(Orientation side) {
	int ts = side == Left ? -1 : 1;
	BodyPart* upperLeg = new BodyPart("upperLeg", LEG_RATIO / 4, LEG_RATIO, 1);
	BodyPart* lowerLeg = new BodyPart("lowerLeg", LEG_RATIO / 4, LEG_RATIO, 2);
	BodyPart* foot = new BodyPart("foot", HAND_RATIO, HAND_RATIO / 2, 3);

	setVector(upperLeg->offset, ts * TORSO_RATIO/4, - TORSO_RATIO * 0.9, 0);
	upperLeg->rotZ = 180;

	lowerLeg->addChild(foot);
	upperLeg->addChild(lowerLeg);
	root->addChild(upperLeg);
}

void HumanModel::createBody() {
	BodyPart* head = new BodyPart("head", HEAD_RATIO, HEAD_RATIO, 1);
	setVector(head->offset, 0, TORSO_RATIO, 0);
	root->addChild(head);

	createArm(Right);
	createArm(Left);
	
	createLeg(Left);
	createLeg(Right);
}

void HumanModel::getState(double* p) {

};

void HumanModel::setState(double* p) {
	
};

void HumanModel::reset(double time) {

};

void HumanModel::display(GLenum mode) {
	glPushMatrix();
	set_colour(1, 0, 0);
	glTranslated(root->offset[0], root->offset[1], root->offset[2]);

	root->drawRoot();
	
	set_colour(1, 1, 1);
	glPopMatrix();
};

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