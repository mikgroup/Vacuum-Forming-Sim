#ifndef COLLISIONOBJECT_PLANE_H
#define COLLISIONOBJECT_PLANE_H

#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "collisionObject.h"

using namespace nanogui;
using namespace CGL;
using namespace std;

struct Plane : public CollisionObject {
public:
  Plane(const Vector3D &point, const Vector3D &normal, double friction)
      : point(point), normal(normal.unit()), friction(friction) {}

  void render(GLShader &shader);
  void collide(PointMass &pm);
	void update(double frames_per_sec, double simulation_steps);

  Vector3D point;
  Vector3D normal;

  double friction;
};

#endif /* COLLISIONOBJECT_PLANE_H */
