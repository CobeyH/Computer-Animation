#pragma once
#include <util/vector.h>

enum updateType {
	startup,
	newTarget
};

struct SetBobState {
	Vector target;
	updateType mode;
};

