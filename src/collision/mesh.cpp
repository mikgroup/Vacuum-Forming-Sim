#include <cmath>
#include <CGL/vector3D.h>
#include <CGL/quaternion.h>
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/mesh_drawing.h"
#include "../misc/mesh_import.h"
#include "mesh.h"

#include <embree2/rtcore.h>
#include <embree2/rtcore_ray.h>

using namespace nanogui;
using namespace CGL;

static void fill_array(float *f, Vector3D vec) {
	f[0] = vec.x;
	f[1] = vec.y;
	f[2] = vec.z;
}

void Mesh::collide(PointMass &pm) {
	if (pm.stuck)
		return;

	RTCRay ray;
	fill_array(ray.org, pm.last_position_no_vel);
	Vector3D sub = pm.last_position_no_vel - pm.last_position; // this seems wrong
	Vector3D sub_norm = Vector3D(sub);
	sub_norm.normalize();
	fill_array(ray.dir, sub_norm);
	ray.tnear = 0;
	ray.tfar = 1.5 * sub.norm(); // extending the ray causes less artifacts
	ray.geomID = RTC_INVALID_GEOMETRY_ID;
	ray.primID = RTC_INVALID_GEOMETRY_ID;
	ray.mask = -1;
	ray.time = 0;

	double tfar_old = ray.tfar;
	rtcIntersect(scene, ray);
	if (ray.geomID == embree_geomID) {
		pm.position = pm.last_position_no_vel + sub_norm * ray.tfar;
		pm.pinned = false;
		pm.stuck = true;
		if (platen) {
			pm.hit_platen = true;
		}
	}
}

void Mesh::render(GLShader &shader) {
	//fill in
	if (!initialized) {
		Misc::import_mesh(*this, filename);
		initialized = true;
	}
	Misc::draw_mesh(*this, shader);
}

void Mesh::add(Vertex *v) {
	verts.push_back(v);
}

void Mesh::add(Face *t) {
	faces.push_back(t);
}

int Mesh::num_faces() {
	return faces.size();
}

int Mesh::num_verts() {
	return verts.size();
}

void Vertex::translate(Vector3D &t) {
	x += t.x;
	y += t.y;
	z += t.z;

}

void Mesh::translate(Vector3D &t, bool move_center) {
	for (vector<Vertex *>::iterator it = verts.begin(); it != verts.end(); it++) {
		(*it)->translate(t);
	}
	if (move_center)
		center += t;
}

void Vertex::rotate(Quaternion *q) {
	Vector3D rot = q->rotatedVector(Vector3D(x, y, z));
	x = rot.x;
	y = rot.y;
	z = rot.z;
}

void Mesh::rotate(tuple<double, Vector3D> r) {
	Quaternion q;
	q.from_axis_angle(get<1>(r), get<0>(r) * M_PI/180);
	Vector3D v1(1,0,0);
	v1 = q.rotatedVector(v1);
	for (vector<Vertex *>::iterator it = verts.begin(); it != verts.end(); it++) {
		(*it)->rotate(&q);
	}
}

void Vertex::scale(double scale) {
	x *= scale;
	y *= scale;
	z *= scale;
}

void Mesh::scale_mesh(double scale, bool move_center) {
	for (vector<Vertex *>::iterator it = verts.begin(); it != verts.end(); it++) {
		(*it)->scale(scale);
	}
	if (move_center)
		center *= scale;
}

Vector3D Mesh::normal(Face *f) {
	Vector3D v0 = Vector3D(verts[f->v0]->x, verts[f->v0]->y, verts[f->v0]->z);
	Vector3D v1 = Vector3D(verts[f->v1]->x, verts[f->v1]->y, verts[f->v1]->z);
	Vector3D v2 = Vector3D(verts[f->v2]->x, verts[f->v2]->y, verts[f->v2]->z);
	Vector3D v = cross(v0 - v1, v0 - v2);
	v.normalize();
	return v;
}

Vector3D Mesh::min() {
	Vector3D v = Vector3D(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());

	for (vector<Vertex *>::iterator it = verts.begin(); it != verts.end(); it++) {
		v.x = std::min((long double) (*it)->x, v.x);
		v.y = std::min((long double) (*it)->y, v.y);
		v.z = std::min((long double) (*it)->z, v.z);
	}

	return v;
}

Vector3D Mesh::max() {
	Vector3D v = Vector3D(std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min());

	for (vector<Vertex *>::iterator it = verts.begin(); it != verts.end(); it++) {
		v.x = std::max((long double) (*it)->x, v.x);
		v.y = std::max((long double) (*it)->y, v.y);
		v.z = std::max((long double) (*it)->z, v.z);
	}

	return v;

}
