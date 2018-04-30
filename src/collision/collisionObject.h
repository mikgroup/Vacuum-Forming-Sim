#ifndef COLLISIONOBJECT
#define COLLISIONOBJECT

#include <nanogui/nanogui.h>

#include "../clothMesh.h"

using namespace CGL;
using namespace std;
using namespace nanogui;

class CollisionObject {
public:
  virtual void render(GLShader &shader) = 0;
  virtual void collide(PointMass &pm) = 0;
	virtual void update(double frames_per_sec, double simulation_steps) = 0;

private:
  double friction;
};

#endif /* COLLISIONOBJECT */
