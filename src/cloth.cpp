#include <iostream>
#include <math.h>
#include <random>
#include <vector>
#include <fstream>
#include <queue>
#include <unordered_set>
#include <CGL/quaternion.h>
#include <iomanip>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "collision/mesh.h"

using namespace std;
static double sim_time = 0;
static bool hit_platen = false;
static double vacuum_force_curr = 0;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

int index(int x, int y, int num_y) {
	return x * num_y + y;
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
	double width_spacing = width / num_width_points;
	double height_spacing = height / num_height_points;

	double rand_z = ((rand() / ((double) RAND_MAX)) - 0.5) * 1 / 1000;
	double curr_x = 0, curr_y = 0;

	unordered_set<int> pins;
	for (int i = 0; i < pinned.size(); i++) {
		pins.insert(index(pinned[i][0], pinned[i][1], num_width_points));
	}

	// Build plane
	for (int x = 0; x < num_height_points; x++) {
		curr_y = 0;
		for (int y = 0; y < num_width_points; y++) {
			int ind = index(x, y, num_width_points);
			bool is_pinned = pins.count(ind) > 0;
			Vector3D pos;
			if (orientation == HORIZONTAL) {
				pos = Vector3D(curr_x, 0.3, curr_y);
			} else {
				pos = Vector3D(curr_x, curr_y, rand_z);
			}

			PointMass pm_new(pos, is_pinned);
			pm_new.uv = Vector2D(pos.x / height, pos.z / width);
			pm_new.start_uv = Vector2D(pm_new.uv);
			pm_new.index = ind;

			point_masses.push_back(pm_new);

			curr_y += width_spacing;
		}
		curr_x += height_spacing;
	}

	// Springs
	for (int x = 0; x < num_height_points; x++) {
		for (int y = 0; y < num_width_points; y++) {
			int ind = index(x, y, num_width_points);
			
			// structural to the left
			if (x - 1 >= 0) {
				springs.push_back(Spring(&point_masses[ind], &point_masses[index(x - 1, y, num_width_points)], STRUCTURAL));
			}
		
		// structural above
			if (y - 1 >= 0) {
				springs.push_back(Spring(&point_masses[ind], &point_masses[index(x, y - 1, num_width_points)], STRUCTURAL));
			}

		// Shearing spring, diag upper left
			if (x - 1 >= 0 && y - 1 >= 0) {
				springs.push_back(Spring(&point_masses[ind], &point_masses[index(x - 1, y - 1, num_width_points)], SHEARING));
			}

			// Shearing spring, diag upper right
			if (x + 1 < num_height_points && y - 1 >= 0) {
				springs.push_back(Spring(&point_masses[ind], &point_masses[index(x + 1, y - 1, num_width_points)], SHEARING));
			}

			// Bending two to the right
			if (x + 2 < num_height_points) {
				springs.push_back(Spring(&point_masses[ind], &point_masses[index(x+2, y, num_width_points)], BENDING));
			}

			// Bending two above
			if (y - 2 >= 0) {
				springs.push_back(Spring(&point_masses[ind], &point_masses[index(x, y - 2, num_width_points)], BENDING));
			}
		}
	}
}

static Eigen::Matrix<long double, 2, 3> uv_transform(Triangle *tri) {
	// Take a triangle with known uv coordinates
	// Return matrix A such that Ap = uv
	// Transform only applies for triangles in the same plane
	

	Eigen::Matrix<long double, 6, 6> X_mat(6,6);
	Eigen::Matrix<long double, 6, 1> B_mat(6);

	Vector3D p1 = tri->pm1->position;
	Vector3D p2 = tri->pm2->position;
	Vector3D p3 = tri->pm3->position;

	X_mat << 	p1.x, p1.y, p1.z, 0, 0, 0,
						0, 0, 0, p1.x, p1.y, p1.z,
						p2.x, p2.y, p2.z, 0, 0, 0,
						0, 0, 0, p2.x, p2.y, p2.z,
						p3.x, p3.y, p3.z, 0, 0, 0,
						0, 0, 0, p3.x, p3.y, p3.z;
	
	B_mat << 	tri->pm1->uv.x, tri->pm1->uv.y,
						tri->pm2->uv.x, tri->pm2->uv.y,
						tri->pm3->uv.x, tri->pm3->uv.y;

	Eigen::Matrix<long double, 6,1> A_mat = X_mat.colPivHouseholderQr().solve(B_mat);

	//cout << A_mat << endl;

	Eigen::Matrix<long double, 2, 3> A(2, 3);
	A << 	A_mat(0), A_mat(1), A_mat(2),
				A_mat(3), A_mat(4), A_mat(5);
	

	return A;
}

// grabbed from c++ docs
template<class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
    almost_equal(T x, T y, int ulp) {
    // the machine epsilon has to be scaled to the magnitude of the values used
    // and multiplied by the desired precision in ULPs (units in the last place)
    return std::abs(x-y) <= std::numeric_limits<T>::epsilon() * std::abs(x+y) * ulp
    // unless the result is subnormal
           || std::abs(x-y) < std::numeric_limits<T>::min();
}

void Cloth::scale_uvs(double scale) {
	for (int i = 0; i < point_masses.size(); i++) {
		point_masses[i].uv *= scale;
	}
}

void Cloth::translate_uvs(double x, double y) {	
	Vector2D translate(x,y);
	
	for (int i = 0; i < point_masses.size(); i++) {
		point_masses[i].uv += translate;
	}
}

void Cloth::remap_uvs() {
//	ifstream f("result.off");
//
//	std::string format;
//	f >> format;
//
//	int num_verts, num_tris, num_edges;
//	f >> num_verts >> num_tris >> num_edges;
//	cout << num_verts << endl;
//	for (int i = num_verts - 1; i >= 0; i--) {
//		int zero;
//		f >> point_masses[i].uv.x >> point_masses[i].uv.y >> zero;
//	}
//
//	f.close();
	
	ifstream f("result2.off");

	for (int i = 0; i < point_masses.size(); i++) {
		f >> point_masses[i].uv.x >> point_masses[i].uv.y;
	}

	Vector2D new_diag = point_masses[0].uv - point_masses[point_masses.size() - 1].uv;
	Vector2D start_diag = point_masses[0].start_uv - point_masses[point_masses.size() - 1].start_uv;

	double scale = start_diag.norm() / new_diag.norm();
/*
	for (int i = 0; i < point_masses.size(); i++) {
		point_masses[i].uv *= scale;
	}
*/
/*
	Vector2D translate = point_masses[0].start_uv - point_masses[0].uv;
	
	for (int i = 0; i < point_masses.size(); i++) {
		point_masses[i].uv += translate;
	}
*/
	f.close();
}

//void Cloth::remap_uvs() {
//	// do the first one
//
//	int middle = index(num_height_points / 2, num_width_points / 2, num_width_points);
//	
//	PointMass p_mid = point_masses[middle];
//	
//	PointMass *pm1 = p_mid.halfedge->triangle->pm1;
//	PointMass *pm2 = p_mid.halfedge->triangle->pm2;
//	PointMass *pm3 = p_mid.halfedge->triangle->pm3;
//
//	pm1->uv = pm1->start_uv; // keep the first point stationary
//	// second point is scaled along edge of uv between pm2 and pm1. Proportional
//	// to ratio of final to start
//	pm2->uv = ((pm2->position - pm1->position).norm() / (pm2->start_position - pm1->start_position).norm()) * (pm2->start_uv - pm1->start_uv) + pm1->start_uv;
//	long double angle_uv_new_1, angle_uv_new_2;
//	angle_uv_new_1 = acos(dot((pm2->position - pm1->position).unit(), (pm3->position - pm1->position).unit()));
//	angle_uv_new_2 = acos(dot((pm1->position - pm2->position).unit(), (pm3->position - pm2->position).unit()));
//
//	long double scale = (pm2->uv - pm1->uv).norm() * sin(angle_uv_new_1) / sin(M_PI - angle_uv_new_1 - angle_uv_new_2);
//	cout << "scale: " << scale << endl;
//	long double theta = -angle_uv_new_2;
//	cout << "theta: " << theta << endl;
//	
////	Eigen::Matrix<long double, 2, 2> rot;
////	Eigen::Matrix<long double, 2, 1>
////	rot << cos(theta), -sin(theta), sin(theta), cos(theta);
//
//	Vector2D dir = Vector2D(dot(Vector2D(cos(theta), -sin(theta)), (pm1->uv - pm2->uv).unit()), dot(Vector2D(sin(theta), cos(theta)), (pm1->uv - pm2->uv).unit()));
//
//	cout << "dir: " << dir << endl;
//	cout << "scale * dir: " << scale * dir << endl;
//	cout << "pm2->uv: " << pm2->uv << endl;
//	cout << "scale * dir + pm2->uv: " << scale * dir + pm2->uv << endl;
//
//	pm3->uv =  scale * dir + pm2->uv;
//
//	cout << acos(dot((pm3->uv - pm1->uv).unit(), (pm2->uv - pm3->uv).unit())) << endl;
//	cout << acos(dot((pm3->position - pm1->position).unit(), (pm2->position - pm3->position).unit())) << endl;
//
//	cout << pm1->uv << pm2->uv << pm3->uv << endl;
//	cout << pm1->start_uv << pm2->start_uv << pm3->start_uv << endl;
//	cout << pm1->position << pm2->position << pm3->position << endl;
//	cout << pm1->start_position << pm2->start_position << pm3->start_position << endl;
//
//
//	// Breadth first search of neighbors with update
//	unordered_set<Triangle *> visited;
//	Triangle *init_tri = p_mid.halfedge->triangle;
//
//	visited.insert(init_tri);
//	
//	queue<Triangle *> queue;
//	// push on initial tri
//	queue.push(init_tri);
//
//	
//	int count = 0;
//
//	while(!queue.empty()) {
//		Triangle *curr_tri = queue.front();
//		Vector3D p1 = curr_tri->halfedge->pm->position;
//		Vector3D p2 = curr_tri->halfedge->next->pm->position;
//		Vector3D p3 = curr_tri->halfedge->next->next->pm->position;
//		
//		Vector3D n1 = cross(p2 - p1, p3 - p1).unit();
//		queue.pop();
//
//		Halfedge *start = curr_tri->halfedge;
//		Halfedge *h = start;
//
//		int c = 1;
//		do {
//			assert(h->next->next->next == h);
//			if (h->twin == nullptr) {
//				h = h->next;
//				continue;
//			}
//
//
//			Triangle *neighbor_tri = h->twin->triangle;
//			Eigen::Matrix<long double, 2, 3> A = uv_transform(curr_tri);
//			
//
//			if (visited.find(neighbor_tri) == visited.end()) { // not visited
//				visited.insert(neighbor_tri);
//				queue.push(neighbor_tri);
//
//				if (!almost_equal(h->twin->next->next->pm->uv.x, h->twin->next->next->pm->start_uv.x,2) || !almost_equal(h->twin->next->next->pm->uv.y, h->twin->next->next->pm->start_uv.y,2)) {
//					//cout << "start uv: " << h->twin->next->next->pm->start_uv << " , curr uv: " << h->twin->next->next->pm->uv << endl;
//				}
//
//
//				// make quaternion from axis angle
//				p1 = h->pm->position;
//				p2 = h->next->pm->position;
//				p3 = h->next->next->pm->position;
//				Vector3D p4 = h->twin->next->next->pm->position;
//
//
//				Vector3D n2 = cross(p2 - p1, p4 - p1).unit();
//
//				// Rotate unknown coordinate to be in the plane of the triangle
//				Quaternion quat;
//
//				long double angle = PI - acos(dot(n1,n2));
//				if (isnan(angle))
//					angle = 0;
//
//				quat.from_axis_angle(p2 - p1, angle);
//				
//				Vector3D p4_prime = quat.rotatedVector(p4 - p1) + p1;
//
//				//Check if actually in plane, otherwise rotate the other direction
//				bool print = false;
//				Vector3D n2_prime = cross(p2 - p1, p4_prime - p1).unit();
//
//				if (!almost_equal(abs(dot(n1, n2_prime)), 1.0l, 2)) {
//					quat.from_axis_angle(p2 - p1, -angle);
//					p4_prime = quat.rotatedVector(p4 - p1) + p1;
//					//cout << "wrong angle 1" << endl;
//
//					n2_prime = cross(p2 - p1, p4_prime - p1).unit();
//				}
//				
//				if (!almost_equal(abs(dot(n1,n2_prime)), 1.0l, 2)) {
//					quat.from_axis_angle(p2 - p1, PI - angle);
//					p4_prime = quat.rotatedVector(p4 - p1) + p1;
//					cout << "wrong angle 2" << endl;
//					n2_prime = cross(p2 - p1, p4_prime - p1).unit();
//					
//					if (!almost_equal(abs(dot(n1,n2_prime)), 1.0l, 2)) {
//						cout << "still not correct angle" << endl;
//						print = true;
//					}
//				}
//
//
//				// Now we have a p4_prime that is in the plane of the known triangle
//				// use A matrix to compute uv
//				Eigen::Matrix<long double, 3, 1> p4_prime_eigen;
//				p4_prime_eigen << p4_prime.x, p4_prime.y, p4_prime.z;
//				Eigen::Matrix<long double, 2, 1> uv_new_eigen = A * p4_prime_eigen;
//				h->twin->next->next->pm->uv = Vector2D(uv_new_eigen(0), uv_new_eigen(1));
//				Vector2D b4(uv_new_eigen(0), uv_new_eigen(1));
//
//				Vector2D b1 = h->pm->uv;
//				Vector2D b2 = h->next->pm->uv;
//				Vector2D b3 = h->next->next->pm->uv;
//				long double tri_area_ratio = cross(p4_prime - p1, p2 - p1).norm() / cross(p3 - p1, p2 - p1).norm();
//				long double uv_area_ratio = abs(cross(b4 - b1, b2 - b1)) / abs(cross(b3 - b1, b2 - b1));
//
//				if (abs(acos(dot((p1 - p4_prime).unit(), (p2-p4_prime).unit())) - acos(dot((b1 - b4).unit(), (b2-b4).unit()))) > 0.00001l) {
//					print = true;
//				}
//				
//				if (print) {
//				//if (!almost_equal(tri_area_ratio, uv_area_ratio,5)) {
//					//cout << "x1 " << p1 << endl;
//					//cout << "x2 " << p2 << endl;
//					//cout << "x3 " << h->next->next->pm->position << endl;
//
//
//					//cout << "b1 " << h->pm->uv << endl;
//					//cout << "b2 " << h->next->pm->uv << endl;
//					//cout << "b3 " << h->next->next->pm->uv << endl;
//
//					//cout << "x4 " << h->twin->next->next->pm->position << endl;
//					//cout << "x4'" << p4_prime << endl;
//
//					cout << "x1 = [" << p1.x << ", " << p1.y << ", " << p1.z << "];" << endl;
//					cout << "x2 = [" << p2.x << ", " << p2.y << ", " << p2.z << "];" << endl;
//					cout << "x3 = [" << p3.x << ", " << p3.y << ", " << p3.z << "];" << endl;
//					cout << "x4 = [" << p4.x << ", " << p4.y << ", " << p4.z << "];" << endl;
//					cout << "b1 = [" << b1.x << ", " << b1.y << "];" << endl;
//					cout << "b2 = [" << b2.x << ", " << b2.y << "];" << endl;
//					cout << "b3 = [" << b3.x << ", " << b3.y << "];" << endl;
//
//					cout << "x4'" << p4_prime << endl;
//					cout << "n1 " << n1 << endl;
//					cout << "n2 " << n2 << endl;
//					cout << "n2_prime " << n2_prime << endl;
//					cout << "quat " << quat << endl;
//					cout << "angle " << angle << endl;
//
//					cout << cross(b2 - b1, b4 - b1) << endl;
//					cout << cross(b2 - b1, b3 - b1) << endl;
//
//
//					cout << "uv " << h->twin->next->next->pm->uv << endl;
//					cout << "A_matrix: " << endl;
//					cout << A << endl;
//
//					cout << "tri area ratio: " <<  tri_area_ratio << endl;
//					cout << "uv  area ratio: " << uv_area_ratio << endl;
//					cout << " " << endl;
//				}
//				long double area_uv1 = cross(b2 - b1, b3 - b1);
//				long double area_uv2 = cross(b2 - b1, b4 - b1);
//
//				//cout << (signbit(area_uv1) != signbit(area_uv2) ? "good" : "bad") << endl;
//
//				//cout << b1.x << ", " << b1.y << ", " << b2.x << ", " << b2.y << ", " << b3.x << ", " << b3.y << endl;
//
//
//			}
//			h = h->next;
//			c++;
//		} while (h != start);  
//		count++;
//	}
//}


void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;
	sim_time += delta_t;

	//reset all forces
	for (int i = 0; i < point_masses.size(); i++) {
		point_masses[i].forces = Vector3D(0,0,0);		
	}

  // TODO (Part 2.1): Compute total force acting on each point mass.
	// f = ma
	Vector3D total_ext_accelerations = Vector3D(0,0,0);
	for (int i = 0; i < external_accelerations.size(); i++)
		total_ext_accelerations += external_accelerations[i];

	for (int i = 0; i < point_masses.size(); i++) {
		if (!point_masses[i].stuck) {
			point_masses[i].forces += mass * total_ext_accelerations;
		}
		if (!hit_platen && point_masses[i].hit_platen) {
			hit_platen = true;
			vacuum_delay += sim_time;
		}
	}
	
	double surf_area = 0;
	if (hit_platen && sim_time > vacuum_delay && abs(vacuum_force) > 0.000001) {
		velocity = Vector3D(0,0,0);
		// vacuum forces
		for (int i = 0; i < clothMesh->triangles.size(); i++) {
			Triangle *tri = clothMesh->triangles[i];
			double area = tri->area();
			double vac = vacuum_force_curr * area;
			surf_area += area;
			Vector3D normal = tri->normal();
			
			if (!tri->pm1->stuck)
				tri->pm1->forces += vac * normal;
			if (!tri->pm2->stuck)
				tri->pm2->forces += vac * normal;
			if (!tri->pm3->stuck)
				tri->pm3->forces += vac * normal;
		}
		if (vacuum_force_curr < vacuum_force)
			vacuum_force_curr += 20;
	}
	
	for (int i = 0; i < springs.size(); i++) {
		Vector3D sub_pos = springs[i].pm_a->position - springs[i].pm_b->position;
		double dist = sub_pos.norm();
		double f_spring = cp->ks * (dist - springs[i].rest_length);
		springs[i].pm_a->forces += -f_spring * sub_pos.unit();
		springs[i].pm_b->forces += f_spring * sub_pos.unit();
	}

	// Velocity
	// dirty trick

	for (int i = 0; i < point_masses.size(); i++) {
		point_masses[i].last_position_no_vel = point_masses[i].last_position;
		if (sim_time > velocity_delay && !point_masses[i].stuck) {
			point_masses[i].last_position += delta_t * velocity;
			point_masses[i].position += delta_t * velocity;
		}
	}

  // TODO (Part 2): Use Verlet integration to compute new point mass positions

	for (int i = 0; i < point_masses.size(); i++) {
		Vector3D curr_last_position = point_masses[i].last_position;
		point_masses[i].last_position = point_masses[i].position;

		if (!point_masses[i].pinned && !point_masses[i].stuck) {
			point_masses[i].position += (1 - cp->damping) * (point_masses[i].position - curr_last_position) + (point_masses[i].forces / mass) * delta_t * delta_t; 
		}
	}
	
	// spring length correction
	for (int i = 0; i < springs.size(); i++) {
		Vector3D sub_pos = springs[i].pm_a->position - springs[i].pm_b->position;
		double dist = sub_pos.norm();
		if (dist > 1.1 * springs[i].rest_length) {
			//correct
			double correct_len = dist - 1.1 * springs[i].rest_length;

			if (!springs[i].pm_a->pinned && !springs[i].pm_b->pinned) {
				// half of correction to each spring
				springs[i].pm_a->position -= correct_len/2 * sub_pos.unit();
				springs[i].pm_a->position += correct_len/2 * sub_pos.unit();
			} else if (springs[i].pm_a->pinned) {
				// apply full correction to point B
				springs[i].pm_b->position += correct_len * sub_pos.unit();
				
			} else if (springs[i].pm_b->pinned) {
				// apply full correction to point A
				springs[i].pm_a->position -= correct_len * sub_pos.unit();
			}
		}

	}
	

  // TODO (Part 4): Handle self-collisions.
  // This won't do anything until you complete Part 4.
  build_spatial_map();
  for (PointMass &pm : point_masses) {
    self_collide(pm, simulation_steps);
  }


  // TODO (Part 3): Handle collisions with other primitives.
  // This won't do anything until you complete Part 3.
  for (PointMass &pm : point_masses) {
    for (CollisionObject *co : *collision_objects) {
      co->collide(pm);
    }
  }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.

}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents
  // membership in some uniquely identified 3D box volume.

  return 0.f;
}


void Cloth::write_to_file(const char *filename) {
	ofstream f(filename);

//	f << "# " << num_width_points << ", " << num_height_points << endl;
//
//	for (int x = 0; x < num_height_points; x++) {
//		for (int y = 0; y < num_width_points; y++) {
//			int ind = index(x, y, num_width_points);
//			f << point_masses[ind].position.x << "," << point_masses[ind].position.z << "," << point_masses[ind].position.y << endl;
//		}
//	}

	f << "OFF" << endl;
	f << num_width_points * num_height_points << " " << clothMesh->triangles.size() << " " << "0" << endl;

	for (int ind = 0; ind < point_masses.size(); ind++) {
			f << point_masses[ind].position.x << " " << point_masses[ind].position.z << " " << point_masses[ind].position.y << endl;
	}

	for (auto tri : clothMesh->triangles) {
		f << 3 << " " << tri->pm1->index << " " << tri->pm2->index << " " << tri->pm3->index << endl;	
	}

	f.close();
}

void Cloth::write_to_svg(string filename, string pngname) {

	// Rescale width and height to a fixed standard to give proj1
	double rescale = width > height ? 512.0 / width : 512.0 / height;
	double newWidth = width * rescale;
	double newHeight = height * rescale;

	ofstream f(filename);

	// Header text
	f << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << endl;
	f << "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">" << endl;
	f << "<svg version=\"1.1\" id=\"Layer_1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" " <<
	      "x=\"0px\" y=\"0px\" width=\"" << newHeight << "px\" height=\"" << newWidth << "px\" viewBox=\"0 0 " << newHeight <<
	      " " << newWidth << "\" enable-background=\"new 0 0 " << newHeight << " " <<  newWidth <<"\" " << "xml:space=\"preserve\">" << endl;
  f << "<texture filename=\"" << pngname << "\" texid=\"map\"/>" << endl;

  // Iterate through all triangles
  for (int i = 0; i < clothMesh->triangles.size(); i++) {
  	Triangle *tri = clothMesh->triangles[i];

  	/*if (tri->pm1->uv.x > 1 || tri->pm1->uv.y > 1
  		  || tri->pm2->uv.x > 1 || tri->pm2->uv.y > 1
  		  || tri->pm3->uv.x > 1 || tri->pm3->uv.y > 1)
  		continue;*/

  	// UV coordinates
  	f << "<textri texid=\"map\" uvs=\"";
  	f << setprecision(15) << tri->pm1->uv.x - floor(tri->pm1->uv.x) << " ";
  	f << setprecision(15) << tri->pm1->uv.y - floor(tri->pm1->uv.y) << " ";
  	f << setprecision(15) << tri->pm2->uv.x - floor(tri->pm2->uv.x) << " ";
  	f << setprecision(15) << tri->pm2->uv.y - floor(tri->pm2->uv.y) << " ";
  	f << setprecision(15) << tri->pm3->uv.x - floor(tri->pm3->uv.x) << " ";
  	f << setprecision(15) << tri->pm3->uv.y - floor(tri->pm3->uv.y) << "\" ";

  	/*f << setprecision(15) << tri->pm1->uv.x << " " << setprecision(15) << tri->pm1->uv.y << " ";
  	f << setprecision(15) << tri->pm2->uv.x << " " << setprecision(15) << tri->pm2->uv.y << " ";
  	f << setprecision(15) << tri->pm3->uv.x << " " << setprecision(15) << tri->pm3->uv.y << "\" ";*/

  	// Points (removing the y as we assume the cloth/plastic starts off perfectly horizontal)
  	f << "points=\"";
  	f << setprecision(15) << rescale * tri->pm1->start_position.x << " ";
  	f << setprecision(15) << rescale * tri->pm1->start_position.z << " ";
  	f << setprecision(15) << rescale * tri->pm2->start_position.x << " ";
  	f << setprecision(15) << rescale * tri->pm2->start_position.z << " ";
  	f << setprecision(15) << rescale * tri->pm3->start_position.x << " ";
  	f << setprecision(15) << rescale * tri->pm3->start_position.z << "\"/>" << endl;
  }

  f << "</svg>" << endl;

	f.close();
}


void Cloth::reset() {
  PointMass *pm = &point_masses[0];
	sim_time = 0;
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
		pm->stuck = false;
		pm->pinned = pm->start_pinned;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm, pm + num_width_points, pm + 1));
      triangles.push_back(new Triangle(pm + 1, pm + num_width_points,
                                       pm + num_width_points + 1));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1;// = new Halfedge();
    Halfedge *h2;// = new Halfedge();
    Halfedge *h3;// = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();


    // Assign halfedge pointers to point masses
//		if (t->pm1->halfedge != nullptr)
 //   	h1 = t->pm1->halfedge;
	//	else
			h1 = new Halfedge();
		t->pm1->halfedge = h1;

//		if (t->pm2->halfedge != nullptr)
	//		h2 = t->pm2->halfedge;
	//	else
			h2 = new Halfedge();
    t->pm2->halfedge = h2;

		//if (t->pm3->halfedge != nullptr)
		//	h3 = t->pm3->halfedge;
		//else
			h3 = new Halfedge();
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

	int count = 0;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->halfedge->twin = temp->halfedge->next->next;
				temp->halfedge->next->next->twin = t->halfedge;
      } else {
        t->halfedge->twin = nullptr;
      }
			t->halfedge->next->twin = triangles[i+1]->halfedge; // always exists
			triangles[i+1]->halfedge->twin = t->halfedge->next;

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->halfedge->next->next->twin = temp->halfedge->next;
        temp->halfedge->next->twin = t->halfedge->next->next;
      } else {
        t->halfedge->next->next->twin = nullptr;
      }
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->halfedge->next->next->twin = temp->halfedge;
        temp->halfedge->twin = t->halfedge->next->next; 
      } else {
        t->halfedge->next->next->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->halfedge->next->twin = temp->halfedge->next->next;
        temp->halfedge->next->next->twin = t->halfedge->next;
      } else {
        t->halfedge->next->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->halfedge->twin = temp->halfedge->next;
			temp->halfedge->next->twin = t->halfedge;
    }

    topLeft = !topLeft;
  }

/*
  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
				temp->pm3->halfedge->twin = t->pm1->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }
			//t->halfedge->next->twin = triangles[i+1]->halfedge;
			//triangles[i+1]->halfedge->twin = t->halfedge->next;

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
				temp->pm2->halfedge->twin = t->pm3->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
			temp->pm1->halfedge->twin = t->pm2->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
				temp->pm1->halfedge->twin = t->pm3->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
				temp->pm3->halfedge->twin = t->pm2->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
			temp->pm2->halfedge->twin = t->pm1->halfedge;
    }

    topLeft = !topLeft;
			Triangle *init_tri = triangles[i];
		cout << "tri " << i << ": " <<  init_tri->halfedge->twin << " " << init_tri->halfedge->next->twin << " " << init_tri->halfedge->next->next->twin << endl;

  }
*/
  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
