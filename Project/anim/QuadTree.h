#pragma once
#include <util/vector.h>
#include "BaseSystem.h"
#include <vector>
#include "States.h"

struct Circle {
	Circle(double xPos, double yPos, double radius) {
		x = xPos;
		y = yPos;
		r = radius;
	}
	double x, y, r;
};

class QuadTree : public BaseSystem {
	public:
		QuadTree(const std::string& name, double boxLength, Vector origin);
		bool insert(Boid boid);
		bool contains(Boid boid);
		void subdivide();
		void query(Circle c, std::vector<Boid> foundBoids);
		bool intersects(Circle);
	protected:
		// Children
		QuadTree* northWest;
		QuadTree* northEast;
		QuadTree* southWest;
		QuadTree* southEast;

		std::vector<Boid> containedBoids;
		int capacity;
		bool divided;
		double length;
		Vector center;

};
