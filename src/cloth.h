#ifndef CLOTH_H
#define CLOTH_H

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "clothMesh.h"
#include "collision/collisionObject.h"
#include "spring.h"

using namespace CGL;
using namespace std;

enum e_orientation { HORIZONTAL = 0, VERTICAL = 1 };

struct ClothParameters {
  ClothParameters() {}
  ClothParameters(bool enable_structural_constraints,
                  bool enable_shearing_constraints,
                  bool enable_bending_constraints, double damping,
                  double density, double ks)
      : enable_structural_constraints(enable_structural_constraints),
        enable_shearing_constraints(enable_shearing_constraints),
        enable_bending_constraints(enable_bending_constraints),
        damping(damping), density(density), ks(ks) {}
  ~ClothParameters() {}

  // Global simulation parameters

  bool enable_structural_constraints;
  bool enable_shearing_constraints;
  bool enable_bending_constraints;

  double damping;

  // Mass-spring parameters
  double density;
  double ks;
};

struct Cloth {
  Cloth() {}
  Cloth(double width, double height, int num_width_points,
        int num_height_points, float thickness);
  ~Cloth();

  void buildGrid();

  void simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects);
  // Implicit Euler functions
  void computeJacobians(ClothParameters* cp);
  void multiplydfdx(vector<Vector3D>& src, vector<Vector3D>& dst);
  void multiplydfdv(vector<Vector3D>& src, vector<Vector3D>& dst);
  void multiplyMatrix(vector<Vector3D>& src, vector<Vector3D>& dst);
  void multiplyMass(ClothParameters* cp, vector<Vector3D>& src, vector<Vector3D>& dst);
  void multiplyScalar(double scal, vector<Vector3D>& src, vector<Vector3D>& dst);
  vector<Vector3D> addVecs(vector<Vector3D>& a, vector<Vector3D>& b);
  void multiplyA(ClothParameters* cp, double dt, vector<Vector3D>& src, vector<Vector3D>& dst);
  void solveSystem(); // Conjugate gradient

  void reset();
	void write_to_file(const char *filename);
  void write_to_svg(string filename, string pngname, double newWidth, double newHeight, double rescale);
  void buildClothMesh();
	void remap_uvs();
	void translate_uvs(double x, double y);
	void rotate_uvs(double theta);
	void scale_uvs(double scale);

  void build_spatial_map();
  void self_collide(PointMass &pm, double simulation_steps);
  float hash_position(Vector3D pos);

  // Cloth properties
  double width;
  double height;
  int num_width_points;
  int num_height_points;
  double thickness;
	string texture;
  e_orientation orientation;
	Vector3D velocity;
	double velocity_delay;
	double vacuum_force;
	double vacuum_delay;

  // Cloth components
  vector<PointMass> point_masses;
  vector<vector<int>> pinned;
  vector<Spring> springs;
  ClothMesh *clothMesh;

  // Spatial hashing
  unordered_map<float, vector<PointMass *> *> map;

	private:
	Vector2D translate_uvs_to_center();
};

#endif /* CLOTH_H */
