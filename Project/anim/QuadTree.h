#pragma once
#include <util/vector.h>
#include "BaseSystem.h"
#include <vector>
#include <list>
#include "States.h"

struct Circle {
	Circle(double xPos, double yPos, double radius) {
		x = xPos;
		y = yPos;
		r = radius;
	}
	double x, y, r;
};

template <class T>
class QuadTree : public BaseSystem {
	public:
		QuadTree(const std::string& name, double boxLength, Vector origin);
		bool insert(T* boid);
		bool contains(T* boid);
		void subdivide();
		void query(Circle* c, T* foundBoids[], int &i);
		bool intersects(Circle* c);
		void display(GLenum mode = GL_RENDER);
		void freeChildren();

	protected:
		// Children
		QuadTree* northWest;
		QuadTree* northEast;
		QuadTree* southWest;
		QuadTree* southEast;

		T* containedBoids[5];
		int amountFilled;
		int capacity;
		bool divided;
		double length;
		Vector center;

};

template class QuadTree<Boid>;
template class QuadTree<Food>;