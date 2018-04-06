#include <cmath>
#include <nanogui/nanogui.h>

#include "mesh_drawing.h"
#include "../collision/mesh.h"

#include "CGL/color.h"
#include "CGL/vector3D.h"

// Static data describing points on a sphere
using namespace nanogui;

namespace CGL {
namespace Misc {

void draw_mesh(Mesh &mesh, GLShader &shader) {
  Matrix4f model;
	model.setIdentity();
  shader.setUniform("model", model);
	
	int num_faces = mesh.num_faces();
  MatrixXf positions(3, 3 * num_faces);
  MatrixXf normals(3, 3 * num_faces);
	int i = 0;
  for (vector<Face *>::iterator it = mesh.faces.begin(); it != mesh.faces.end(); it++) {
		Vertex *v0 = mesh.verts.at((*it)->v0); 
		Vertex *v1 = mesh.verts.at((*it)->v1); 
		Vertex *v2 = mesh.verts.at((*it)->v2); 
    Vector3D p0(v0->x,v0->y,v0->z);
    Vector3D p1(v1->x,v1->y,v1->z);
    Vector3D p2(v2->x,v2->y,v2->z);

    positions.col(i) << p0.x, p0.y, p0.z;
    positions.col(i + 1) << p1.x, p1.y, p1.z;
    positions.col(i + 2) << p2.x, p2.y, p2.z;

		Vector3D normal = mesh.normal(*it);
		normals.col(i) << normal.x, normal.y, normal.z;
		normals.col(i + 1) << normal.x, normal.y, normal.z;
		normals.col(i + 2) << normal.x, normal.y, normal.z;
		i += 3;

  }
  shader.uploadAttrib("in_position", positions);
  shader.uploadAttrib("in_normal", normals);

  shader.drawArray(GL_TRIANGLES, 0, num_faces * 3);
}

} // namespace Misc
} // namespace CGL
