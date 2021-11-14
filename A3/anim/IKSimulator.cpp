#include "IKSimulator.h"
#include "Hermite.h"

IKSimulator::IKSimulator(const std::string& name, BaseSystem* target) : BaseSimulator(name), m_object(target) {
	
};

void IKSimulator::registerHermite(Hermite* hermite) {
	tracedPath = hermite;
}

int IKSimulator::step(double time) {
	return -1;
};

int IKSimulator::command(int argc, myCONST_SPEC char** argv) {
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}

	else if (strcmp(argv[0], "read") == 0) {
		if (argc != 2) {
			animTcl::OutputMessage("Invalid arguments passed. Expeced 2 arguments but got %d", argc);
			return TCL_ERROR;
		}
		tracedPath->loadFromFile2D(argv[1]);
		glutPostRedisplay();
	}
	else {
		animTcl::OutputMessage("The given command %s is not valid", argv[0]);
	}
};