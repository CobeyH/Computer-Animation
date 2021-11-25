#include "QuadTree.h"
#include "BaseSystem.h"
	
template <typename T>
QuadTree<T>::QuadTree(const std::string& name, double boxLength, Vector origin) : BaseSystem(name) {
	capacity = 5;
	amountFilled = 0;
	divided = false;
	length = boxLength;
	VecCopy(center, origin);
	northWest = NULL;
}

template <typename T>
bool QuadTree<T>::contains(T* boid) {
	double x = boid->position[0];
	double y = boid->position[1];
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

template <typename T>
bool QuadTree<T>::insert(T* boid) {
	if (!contains(boid)) {
		return false;
	}
	// If there is room in the current quad then just insert it.
	if (amountFilled < capacity) {
		containedBoids[amountFilled] = boid;
		amountFilled++;
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

template <typename T>
void QuadTree<T>::subdivide() {
	double currX = center[0];
	double currY = center[1];
	double newLength = length / 2;
	double newRadius = newLength / 2;

	Vector newCenter;
	setVector(newCenter, currX + newRadius, currY + newRadius, 0);
	northWest = new QuadTree("northwest", newLength, newCenter);

	setVector(newCenter, currX - newRadius, currY + newRadius, 0);
	northEast = new QuadTree("northEast", newLength, newCenter);

	setVector(newCenter, currX + newRadius, currY - newRadius, 0);
	southWest = new QuadTree("southWest", newLength, newCenter);

	setVector(newCenter, currX - newRadius, currY - newRadius, 0);
	southEast = new QuadTree("southEast", newLength, newCenter);

	divided = true;
}

template <typename T>
bool QuadTree<T>::intersects(Circle* c) {
	double differenceX = abs(c->x - center[0]);
	double differenceY = abs(c->y - center[1]);
	double l2 = length / 2;

	// There is no way they can overlap if the distance is greater than half 
	// the square length + radius of circle
	if (differenceX > l2 + c->r || differenceY > l2 + c->r) {
		return false;
	}
	if (differenceX <= l2 || differenceY <= l2) {
		return true;
	}
	return false;
}

template<typename T>
void QuadTree<T>::query(Circle* c, T* foundBoids[], int &i) {
	if (!intersects(c)) {
		return;
	}
	for (int index = 0; index < amountFilled; index++) {
		if (abs(c->x - containedBoids[index]->position[0]) + abs(c->y - containedBoids[index]->position[1]) <= c->r) {
			foundBoids[i++] = containedBoids[index];
		}
	}
	// If there are no children
	if (northWest == NULL) {
		return;
	}
	northWest->query(c, foundBoids, i);
	northEast->query(c, foundBoids, i);
	southWest->query(c, foundBoids, i);
	southEast->query(c, foundBoids, i);
}

template<typename T>
void QuadTree<T>::display(GLenum mode) {

	if (!divided) {
		glBegin(GL_LINE_LOOP);
		glVertex3d(center[0] + length/2, center[1] + length/2, 0);
		glVertex3d(center[0] + length / 2, center[1] - length / 2, 0);
		glVertex3d(center[0] - length / 2, center[1] - length / 2, 0);
		glVertex3d(center[0] - length / 2, center[1] + length / 2, 0);
		glEnd();
	}
	else {
		northWest->display(mode);
		northEast->display(mode);
		southWest->display(mode);
		southEast->display(mode);
	}
}

template<typename T>
void QuadTree<T>::freeChildren() {
	if (divided) {
		northWest->freeChildren();
		northEast->freeChildren();
		southWest->freeChildren();
		southEast->freeChildren();
		
	}
	delete(this);
}