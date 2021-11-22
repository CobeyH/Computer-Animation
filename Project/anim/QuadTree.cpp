#include "QuadTree.h"
#include "BaseSystem.h"
	
QuadTree::QuadTree(const std::string& name, double boxLength, Vector origin) : BaseSystem(name) {
	capacity = 5;
	divided = false;
	length = boxLength;
	VecCopy(center, origin);
	northWest = NULL;
}

bool QuadTree::contains(Boid boid) {
	double x = boid.position[0];
	double y = boid.position[1];
	double radius = length / 2;
	// Out of bounds in X direction
	if (x < center[0] - radius || x > center[0] + radius) {
		return false;
	}
	else if (y < center[1] - radius || y > center[1] + radius) {
		return false;
	}
	return true;
}

bool QuadTree::insert(Boid boid) {
	if (!contains(boid)) {
		return false;
	}
	// If there is room in the current quad then just insert it.
	if (containedBoids.size() < capacity) {
		containedBoids.push_back(boid);
		return true;
	}

	if (!divided) {
		subdivide();
	}

	return northEast->insert(boid) ||
		   northWest->insert(boid) ||
		   southWest->insert(boid) || 
		   southEast->insert(boid);
}
void QuadTree::subdivide() {
	double currX = center[0];
	double currY = center[1];
	double delta = length / 2;

	Vector newCenter;
	setVector(newCenter, currX + delta, currY + delta, 0);
	northWest = new QuadTree("northwest", delta, newCenter);

	setVector(newCenter, currX - delta, currY + delta, 0);
	northEast = new QuadTree("northEast", delta, newCenter);

	setVector(newCenter, currX + delta, currY - delta, 0);
	southWest = new QuadTree("southWest", delta, newCenter);

	setVector(newCenter, currX - delta, currY - delta, 0);
	southEast = new QuadTree("southEast", delta, newCenter);

	divided = true;
}

bool QuadTree::intersects(Circle c) {
	double differenceX = abs(c.x - center[0]);
	double differenceY = abs(c.y - center[1]);

	// There is no way they can overlap if the distance is greater than half 
	// the square length + radius of circle
	if (differenceX > length / 2 + c.r || differenceY > length / 2 + c.r) {
		return false;
	}
	if (differenceX <= length / 2 || differenceY <= length / 2) {
		return true;
	}
	double cornerDist = pow(differenceX - length / 2, 2) + pow(differenceY - length / 2, 2);

	return cornerDist <= pow(c.r, 2);
}

void QuadTree::query(Circle c, std::vector<Boid> foundBoids) {
	if (!intersects(c)) {
		return;
	}
	for (Boid b : containedBoids) {
		if (contains(b)) {
			foundBoids.push_back(b);
		}
	}
	// If there are no children
	if (northWest == NULL) {
		return;
	}
	northWest->query(c, foundBoids);
	northEast->query(c, foundBoids);
	southWest->query(c, foundBoids);
	southEast->query(c, foundBoids);
}