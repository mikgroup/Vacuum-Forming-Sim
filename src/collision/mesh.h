#ifndef COLLISIONOBJECT_MESH_H
#define COLLISIONOBJECT_MESH_H

#include "../clothMesh.h"
#include "collisionObject.h"
#include <embree3/rtcore.h>
#include <embree3/rtcore_ray.h>

/*
	Generic meshes imported from OBJ files
	Collision dectection handled with ray tracing from Embree
*/

using namespace CGL;
using namespace std;

struct Vertex;
struct Face;

// data structures required by Embree API
struct Vertex {
	float x, y, z, a;
	Vertex(float x, float y, float z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

	void translate(Vector3D &t);
	void scale(double scale);
	void rotate(Quaternion *q);
	
};

// v0, v1, v2 are indicies in vertex list representing one face
struct Face {
	int v0, v1, v2;
	
	Face(int i0, int i1, int i2) {
		v0 = i0;
		v1 = i1;
		v2 = i2;
	}

};


struct Mesh : public CollisionObject {
public:
	Mesh(string filename, const Vector3D &trans, vector<tuple<double, Vector3D>> rotations, double scale, RTCScene &scene, RTCDevice &device, bool platen)
		: filename(filename), trans(trans), rotations(rotations), scale(scale), scene(scene), device(device), platen(platen) {}

	void render(GLShader &shader);
	void collide(PointMass &pm);
	void add(Vertex *v);
	void add(Face *t);
	int num_faces();
	int num_verts();
	void translate(Vector3D &t, bool move_center = true);
	void scale_mesh(double scale, bool move_center = true);
	void rotate(tuple<double, Vector3D>);
	Vector3D normal(Face *f);
	Vector3D min();
	Vector3D max();
	
	vector<Vertex *> verts;
	vector<Face *> faces;

	string filename;
	Vector3D trans;
	vector<tuple<double, Vector3D>> rotations;
	double scale;
	bool initialized = false;
	RTCScene scene;
	RTCDevice device;
	Vector3D center = Vector3D();
	Vector3D cloth_center;
	bool platen;
	unsigned int embree_geomID;
};

#endif /* COLLISION_OBJECT_MESH_H */
