#include <cmath>
#include <fstream>

#include "mesh_import.h"
#include "../collision/mesh.h"
#include <embree3/rtcore.h>
#include <embree3/rtcore_ray.h>

#include "CGL/vector3D.h"

// Some code from ARCSim (http://graphics.berkeley.edu/resources/ARCSim/) 

using namespace std;

namespace CGL {
namespace Misc {

void get_valid_line (istream &in, string &line) {
	do
		getline(in, line);
	while (in && (line.length() == 0 || line[0] == '#'));
}

void import_mesh(Mesh &mesh, string filename) {
	ifstream file;
	file.open(filename);

	if (!file) {
		cerr << "Unable to open file " << filename << " in mesh_import.";
		exit(1);
	}

	while (file) {
		string line;
		get_valid_line(file, line);

		stringstream linestream(line);
		string keyword;
		linestream >> keyword;
		if (keyword == "v") { // Vertex
			Vector3D v;
			linestream >> v[0] >> v[1] >> v[2];
			mesh.center += v;
			mesh.add(new Vertex(v[0], v[1], v[2]));
		} else if (keyword == "vt") { // uv's for texture, not included yet
			//Vector2D uv;
			//linestream >> uv[0] >> uv[1];
		} else if (keyword == "f") {
			string w;
			vector<int> inds;
			while (linestream >> w) {
				stringstream wstream(w);
				int v, n;
				char c;
				wstream >> n >> c >> v;
				inds.push_back(n - 1);
			}
			mesh.add(new Face(inds[0], inds[1], inds[2]));
		}
	}
	file.close();

	mesh.center /= mesh.verts.size();
	
	// Rotations
	Vector3D neg_center = -mesh.center;

	mesh.translate(neg_center, false);
	for (int i = 0; i < mesh.rotations.size(); i++) {
		mesh.rotate(mesh.rotations.at(i));
	}
	mesh.translate(mesh.center, false);


	// Scale and translate
	mesh.scale_mesh(mesh.scale);
	mesh.translate(mesh.trans);

	// Move to center of cloth
	Vector3D sub = mesh.cloth_center - mesh.center;
	mesh.translate(sub);

	// If platen, move top to y coordinate 0
	if (mesh.platen) {
		Vector3D v_plat = Vector3D(0, -mesh.max().y, 0);
		mesh.translate(v_plat);
	} else {
		// move other objects bottom to top of platen
		Vector3D v_ob = Vector3D(0, -mesh.min().y, 0);
		mesh.translate(v_ob);
	}



	// Loading tris and verts into Embree scene
	unsigned int geomID;
 RTCGeometry geom_0 = rtcNewGeometry (mesh.device, RTC_GEOMETRY_TYPE_TRIANGLE); // EMBREE_FIXME: check if geometry gets properly committed
 rtcSetGeometryBuildQuality(geom_0,RTC_BUILD_QUALITY_MEDIUM);
 rtcSetGeometryTimeStepCount(geom_0,1);
 geomID = rtcAttachGeometry(mesh.scene,geom_0);
 rtcReleaseGeometry(geom_0);

	mesh.embree_geomID = geomID;
	
	Vertex *vertices = (Vertex *) rtcSetNewGeometryBuffer(geom_0,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,4*sizeof(float),mesh.verts.size());
	int i = 0;
	for (vector<Vertex *>::iterator it = mesh.verts.begin(); it != mesh.verts.end(); it++) {
		vertices[i].x = (*it)->x;	
		vertices[i].y = (*it)->y;	
		vertices[i].z = (*it)->z;
		i++;
	}
	

	Face *faces = (Face *) rtcSetNewGeometryBuffer(geom_0,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,3*sizeof(int),mesh.faces.size());
	i = 0;
	for (vector<Face *>::iterator it = mesh.faces.begin(); it != mesh.faces.end(); it++) {
		faces[i].v0 = (*it)->v0;
		faces[i].v1 = (*it)->v1;
		faces[i].v2 = (*it)->v2;
		i++;
	}
	
}

} // namespace Misc
} // namespace CGL
