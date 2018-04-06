#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
	double dist_from_origin = (pm.position - origin).norm();
	if (dist_from_origin < radius) {
		Vector3D tangent_point = (pm.position - origin).unit() * radius + origin;
		Vector3D correction = pm.last_position - tangent_point;
		pm.position = pm.last_position - correction * (1-friction);
	}
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  Misc::draw_sphere(shader, origin, radius * 0.92);
}
