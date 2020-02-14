#ifndef POINTMASS_H
#define POINTMASS_H

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "CGL/vector3D.h"
#include "CGL/vector2D.h"

using namespace CGL;

// Forward declarations
class Halfedge;

struct PointMass {
  PointMass(Vector3D position, bool pinned)
      : pinned(pinned), start_position(position), position(position),
        last_position(position) {
					start_pinned = pinned;
				}

  Vector3D normal();
  Vector3D velocity(double delta_t) {
    return (position - last_position) / delta_t;
  }

  // static values
  bool pinned;
	bool start_pinned;
	bool stuck = false;
  unsigned int collide_id = -1; //Which object the point mass is stuck to. -1 if not stuck.
  Vector3D start_position;
	bool hit_platen = false;

	int index = 0;

  // dynamic values
  Vector3D position;
  Vector3D last_position;
	Vector3D last_position_no_vel;
  Vector3D forces;

	Vector2D uv;
	Vector2D start_uv;

  // mesh reference
  Halfedge *halfedge;
};

#endif /* POINTMASS_H */
