# CSC-473
Computer animation course at UVic. This course focusses on the underlying systems that are used in animation instead of the product of art. It leans heavily on the mathimatical methods used to create animation.

## Assignment 1
The first assignment uses Hermite Splines to animate a car along a path. The path is creating by entering points onto a plane. Catmull-Rom initialization with second-order accurate boundary conditions is used to smooth the points to create a continuous path.
Since the car is meant to have continuous velocity, Arc Length parameterization using a piecewise linear approximation and a lookup table is used to calculate the car's position for each animation step. A projectile is fired at the car in the animation, using an ease-in, ease-out motion curve.

## Assignment 2
The second assignment is a Spring Mass System that uses point mass particles and springs to create animations. The system supports an arbitrary number of particles and spring connections that can be entered dynamically. The particle system also supports gravity, ground force and air resistance.
Forward Euler, Symplectic Euler and Verlet integration are all supported.

## Assignment 3
The third assignment uses inverse kinematics to model a human character drawing on a chalkboard. The hermite spline class in re-used to create a shape on the chalkboard for the human model to draw. The end effector of the human is moved along the spline, while the wrist, elbow and shoulder rotate using inverse kinematics to create realistic motion.

## Final Project
The final project had no prescribed topic and allowed the students to choose any area of animation that interested them. For my submission, I created an evolution simulator that uses Boid to model flocking birds. The birds are all initialized with random attributes and evolve over time as they either starve to death or die to a predator boid. Much more detail is provided in the final report latex document.
