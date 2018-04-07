#include <getopt.h>
#include <iostream>
#include <fstream>
#include <nanogui/nanogui.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <unordered_set>
#include <embree3/rtcore.h>
#include <embree3/rtcore_ray.h>

#include "CGL/CGL.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "collision/mesh.h"
#include "cloth.h"
#include "clothSimulator.h"
#include "json.hpp"


typedef uint32_t gid_t;

using namespace std;
using namespace nanogui;

using json = nlohmann::json;

#define msg(s) cerr << "[ClothSim] " << s << endl;

const string SPHERE = "sphere";
const string PLANE = "plane";
const string CLOTH = "cloth";
const string MESH = "mesh";

const unordered_set<string> VALID_KEYS = {SPHERE, PLANE, CLOTH, MESH};

ClothSimulator *app = nullptr;
GLFWwindow *window = nullptr;
Screen *screen = nullptr;

void error_callback(int error, const char* description) {
  puts(description);
}

void createGLContexts() {
  if (!glfwInit()) {
    return;
  }

  glfwSetTime(0);

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  glfwWindowHint(GLFW_SAMPLES, 0);
  glfwWindowHint(GLFW_RED_BITS, 8);
  glfwWindowHint(GLFW_GREEN_BITS, 8);
  glfwWindowHint(GLFW_BLUE_BITS, 8);
  glfwWindowHint(GLFW_ALPHA_BITS, 8);
  glfwWindowHint(GLFW_STENCIL_BITS, 8);
  glfwWindowHint(GLFW_DEPTH_BITS, 24);
  glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

  // Create a GLFWwindow object
  window = glfwCreateWindow(800, 800, "Vacuum Forming Simulation", nullptr, nullptr);
  if (window == nullptr) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    throw std::runtime_error("Could not initialize GLAD!");
  }
  glGetError(); // pull and ignore unhandled errors like GL_INVALID_ENUM

  glClearColor(0.2f, 0.25f, 0.3f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  // Create a nanogui screen and pass the glfw pointer to initialize
  screen = new Screen();
  screen->initialize(window, true);

  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  glViewport(0, 0, width, height);
  glfwSwapInterval(1);
  glfwSwapBuffers(window);
}

void setGLFWCallbacks() {
  glfwSetCursorPosCallback(window, [](GLFWwindow *, double x, double y) {
    if (!screen->cursorPosCallbackEvent(x, y)) {
      app->cursorPosCallbackEvent(x / screen->pixelRatio(),
                                  y / screen->pixelRatio());
    }
  });

  glfwSetMouseButtonCallback(
      window, [](GLFWwindow *, int button, int action, int modifiers) {
        if (!screen->mouseButtonCallbackEvent(button, action, modifiers) ||
            action == GLFW_RELEASE) {
          app->mouseButtonCallbackEvent(button, action, modifiers);
        }
      });

  glfwSetKeyCallback(
      window, [](GLFWwindow *, int key, int scancode, int action, int mods) {
        if (!screen->keyCallbackEvent(key, scancode, action, mods)) {
          app->keyCallbackEvent(key, scancode, action, mods);
        }
      });

  glfwSetCharCallback(window, [](GLFWwindow *, unsigned int codepoint) {
    screen->charCallbackEvent(codepoint);
  });

  glfwSetDropCallback(window,
                      [](GLFWwindow *, int count, const char **filenames) {
                        screen->dropCallbackEvent(count, filenames);
                        app->dropCallbackEvent(count, filenames);
                      });

  glfwSetScrollCallback(window, [](GLFWwindow *, double x, double y) {
    if (!screen->scrollCallbackEvent(x, y)) {
      app->scrollCallbackEvent(x, y);
    }
  });

  glfwSetFramebufferSizeCallback(window,
                                 [](GLFWwindow *, int width, int height) {
                                   screen->resizeCallbackEvent(width, height);
                                   app->resizeCallbackEvent(width, height);
                                 });
}

void usageError(const char *binaryName) {
  printf("Usage: %s [options]\n", binaryName);
  printf("Required program options:\n");
  printf("  -f     <STRING>    Filename of scene");
  printf("\n");
  exit(-1);
}

void incompleteObjectError(const char *object, const char *attribute) {
  cout << "Incomplete " << object << " definition, missing " << attribute << endl;
  exit(-1);
}

void loadObjectsFromFile(string filename, Cloth *cloth, ClothParameters *cp, vector<CollisionObject *>* objects, vector<int> *mesh_inds, RTCScene *scene, RTCDevice *device) {
  // Read JSON from file
  ifstream i(filename);
  json j;
  i >> j;

  // Loop over objects in scene
  for (json::iterator it = j.begin(); it != j.end(); ++it) {
    string key = it.key();

    // Check that object is valid
    unordered_set<string>::const_iterator query = VALID_KEYS.find(key);
    if (query == VALID_KEYS.end()) {
      cout << "Invalid scene object found: " << key << endl;
      exit(-1);
    }

    // Retrieve object
    json object = it.value();

    // Parse object depending on type (cloth, sphere, or plane)
    if (key == CLOTH) {
      // Cloth
      double width, height;
      int num_width_points, num_height_points;
      float thickness;
      e_orientation orientation;
      vector<vector<int>> pinned;
			Vector3D velocity;
			double velocity_delay;
			double vacuum_force;
			double vacuum_delay;

      auto it_width = object.find("width");
      if (it_width != object.end()) {
        width = *it_width;
      } else {
        incompleteObjectError("cloth", "width");
      }

      auto it_height = object.find("height");
      if (it_height != object.end()) {
        height = *it_height;
      } else {
        incompleteObjectError("cloth", "height");
      }

      auto it_num_width_points = object.find("num_width_points");
      if (it_num_width_points != object.end()) {
        num_width_points = *it_num_width_points;
      } else {
        incompleteObjectError("cloth", "num_width_points");
      }

      auto it_num_height_points = object.find("num_height_points");
      if (it_num_height_points != object.end()) {
        num_height_points = *it_num_height_points;
      } else {
        incompleteObjectError("cloth", "num_height_points");
      }

      auto it_thickness = object.find("thickness");
      if (it_thickness != object.end()) {
        thickness = *it_thickness;
      } else {
        incompleteObjectError("cloth", "thickness");
      }

      auto it_orientation = object.find("orientation");
      if (it_orientation != object.end()) {
        orientation = *it_orientation;
      } else {
        incompleteObjectError("cloth", "orientation");
      }

      auto it_pinned = object.find("pinned");
      if (it_pinned != object.end()) {
        vector<json> points = *it_pinned;
        for (auto pt : points) {
          vector<int> point = pt;
          pinned.push_back(point);
        }
      }

			auto it_pin_edges = object.find("pin_edges");
			if (it_pin_edges != object.end()) {
				bool pin_edges = *it_pin_edges;
				if (pin_edges) {
					for (int i = 0; i < num_width_points; i++) {
						pinned.push_back({0,i});
						pinned.push_back({num_height_points - 1, i});
					}

					for (int i = 1; i < num_height_points - 1; i++) {
						pinned.push_back({i, 0});
						pinned.push_back({i, num_width_points - 1});
					}
				}
			}
			
			auto it_vacuum = object.find("vacuum");
			if (it_vacuum != object.end()) {
				json vac_object = *it_vacuum;
				auto it_mag = vac_object.find("magnitude");
				if (it_mag != vac_object.end()) {
					vacuum_force = *it_mag;
				} else {
					incompleteObjectError("cloth", "vacuum::magnitude");
				}

				auto it_delay = vac_object.find("delay");
				if (it_delay != vac_object.end()) {
					vacuum_delay = *it_delay;
				} else {
					vacuum_delay = 0;
				}
			} else {
				vacuum_force = 0;
				vacuum_delay = 0;
			}

			auto it_motion = object.find("motion");
			if (it_motion != object.end()) {
				json vel_object = *it_motion;
				auto it_velocity = vel_object.find("velocity");
				if (it_velocity != vel_object.end()) {
					vector<double> vel = *it_velocity;
					velocity = Vector3D(vel[0], vel[1], vel[2]);

					auto it_delay = vel_object.find("delay");
					if (it_delay != vel_object.end()) {
						velocity_delay = *it_delay;
					} else {
						velocity_delay = 0;
					}
				} else {
					velocity = Vector3D(0,0,0);
					velocity_delay = 0;
				}
			} else {
				velocity = Vector3D(0,0,0);
				velocity_delay = 0;
			}

      cloth->width = width;
      cloth->height = height;
      cloth->num_width_points = num_width_points;
      cloth->num_height_points = num_height_points;
      cloth->thickness = thickness;
      cloth->orientation = orientation;
      cloth->pinned = pinned;
			cloth->velocity = velocity;
			cloth->velocity_delay = velocity_delay;
			cloth->vacuum_force = vacuum_force;
			cloth->vacuum_delay = vacuum_delay;

      // Cloth parameters
      bool enable_structural_constraints, enable_shearing_constraints, enable_bending_constraints;
      double damping, density, ks;

      auto it_enable_structural = object.find("enable_structural");
      if (it_enable_structural != object.end()) {
        enable_structural_constraints = *it_enable_structural;
      } else {
        incompleteObjectError("cloth", "enable_structural");
      }

      auto it_enable_shearing = object.find("enable_shearing");
      if (it_enable_shearing != object.end()) {
        enable_shearing_constraints = *it_enable_shearing;
      } else {
        incompleteObjectError("cloth", "it_enable_shearing");
      }

      auto it_enable_bending = object.find("enable_bending");
      if (it_enable_bending != object.end()) {
        enable_bending_constraints = *it_enable_bending;
      } else {
        incompleteObjectError("cloth", "it_enable_bending");
      }

      auto it_damping = object.find("damping");
      if (it_damping != object.end()) {
        damping = *it_damping;
      } else {
        incompleteObjectError("cloth", "damping");
      }

      auto it_density = object.find("density");
      if (it_density != object.end()) {
        density = *it_density;
      } else {
        incompleteObjectError("cloth", "density");
      }

      auto it_ks = object.find("ks");
      if (it_ks != object.end()) {
        ks = *it_ks;
      } else {
        incompleteObjectError("cloth", "ks");
      }

      cp->enable_structural_constraints = enable_structural_constraints;
      cp->enable_shearing_constraints = enable_shearing_constraints;
      cp->enable_bending_constraints = enable_bending_constraints;
      cp->density = density;
      cp->damping = damping;
      cp->ks = ks;
    } else if (key == SPHERE) {
      Vector3D origin;
      double radius, friction;

      auto it_origin = object.find("origin");
      if (it_origin != object.end()) {
        vector<double> vec_origin = *it_origin;
        origin = Vector3D(vec_origin[0], vec_origin[1], vec_origin[2]);
      } else {
        incompleteObjectError("sphere", "origin");
      }

      auto it_radius = object.find("radius");
      if (it_radius != object.end()) {
        radius = *it_radius;
      } else {
        incompleteObjectError("sphere", "radius");
      }

      auto it_friction = object.find("friction");
      if (it_friction != object.end()) {
        friction = *it_friction;
      } else {
        incompleteObjectError("sphere", "friction");
      }

      Sphere *s = new Sphere(origin, radius, friction);
      objects->push_back(s);
    } else if (key == PLANE) { // PLANE
      Vector3D point, normal;
      double friction;

      auto it_point = object.find("point");
      if (it_point != object.end()) {
        vector<double> vec_point = *it_point;
        point = Vector3D(vec_point[0], vec_point[1], vec_point[2]);
      } else {
        incompleteObjectError("plane", "point");
      }

      auto it_normal = object.find("normal");
      if (it_normal != object.end()) {
        vector<double> vec_normal = *it_normal;
        normal = Vector3D(vec_normal[0], vec_normal[1], vec_normal[2]);
      } else {
        incompleteObjectError("plane", "normal");
      }

      auto it_friction = object.find("friction");
      if (it_friction != object.end()) {
        friction = *it_friction;
      } else {
        incompleteObjectError("plane", "friction");
      }

      Plane *p = new Plane(point, normal, friction);
      objects->push_back(p);
    } else if (key == MESH) {
			Vector3D translate;
			double scale;
			string mesh_filename;

			if (!object.is_array())
				object = json::array({object});
			
			for (json::iterator ob_it = object.begin(); ob_it != object.end(); ob_it++) {
				json ob = *ob_it;
				auto it_trans = ob.find("translate");
				if (it_trans != ob.end()) {
					vector<double> vec_trans = *it_trans;
					translate = Vector3D(vec_trans[0], vec_trans[1], vec_trans[2]);
				} else {
					translate = Vector3D(0,0,0);
				}

				auto it_scale = ob.find("scale");
				if (it_scale != ob.end()) {
					scale = *it_scale;
				} else {
					scale = 1;
				}

				auto it_rot = ob.find("rotate");
				vector<tuple<double, Vector3D>> rotations;
				if (it_rot != ob.end()) {
					// iterate through rotations
					for (json::iterator ob_it_rot = it_rot.value().begin(); ob_it_rot != it_rot.value().end(); ob_it_rot++) {
						json ob_rot = ob_it_rot.value();
						Vector3D axis_vec;
						
						auto it_axis = ob_rot.find("axis");

						if (it_axis != ob_rot.end()) {
							vector<double> axis = *it_axis;
							axis_vec = Vector3D(axis[0], axis[1], axis[2]);
						} else {
							incompleteObjectError("mesh", "rotate::axis");
						}

						auto it_angle = ob_rot.find("angle");
						double angle;
						if (it_angle != ob_rot.end()) {
							angle = *it_angle;
						} else {
							incompleteObjectError("mesh", "rotate::angle");
						}
						rotations.push_back(make_tuple(angle, axis_vec));
					}
				}

				auto it_m_file = ob.find("filename");
				if (it_m_file != ob.end()) {
					mesh_filename = *it_m_file;
				} else {
					incompleteObjectError("mesh", "filename");
				}
				
				bool platen = false;
				auto it_platen = ob.find("platen");
				if (it_platen != ob.end()) {
					platen = *it_platen;
				} else {
					platen = false;
				}
				Mesh *m = new Mesh(mesh_filename, translate, rotations, scale, *scene, *device, platen);
				mesh_inds->push_back(objects->size());
				objects->push_back(m);
			}
		}
  }

  i.close();
}

int main(int argc, char **argv) {
  Cloth cloth;
  ClothParameters cp;
  vector<CollisionObject *> objects;
	vector<int> mesh_inds;

	// Init Embree
	RTCDevice device = rtcNewDevice("");
	RTCScene scene = rtcNewScene(device);
  
	// rtcSetSceneFlags(scene,RTC_BUILD_QUALITY_MEDIUM | RTC_BUILD_QUALITY_HIGH | RTC_SCENE_FLAG_ROBUST); // EMBREE_FIXME: set proper scene flags
  // rtcSetSceneBuildQuality(scene,RTC_BUILD_QUALITY_MEDIUM | RTC_BUILD_QUALITY_HIGH | RTC_SCENE_FLAG_ROBUST); // EMBREE_FIXME: set proper build quality

  if (argc == 1) { // No arguments, default initialization
    string default_file_name = "../scene/pinned2.json";
    loadObjectsFromFile(default_file_name, &cloth, &cp, &objects, &mesh_inds, &scene, &device);
  } else {
    int c;

    while ((c = getopt (argc, argv, "f:")) != -1) {
      switch (c) {
        case 'f':
          loadObjectsFromFile(optarg, &cloth, &cp, &objects, &mesh_inds, &scene, &device);
          break;
        default:
          usageError(argv[0]);
					rtcReleaseScene(scene);
					rtcReleaseDevice (device);
      }
    }
  }

  glfwSetErrorCallback(error_callback);

  createGLContexts();

  // Initialize the Cloth object
  cloth.buildGrid();
  cloth.buildClothMesh();
	
	// Store the center of the cloth in each mesh

	Vector3D cloth_center(0,0,0);
	for (auto &pm : cloth.point_masses) {
		cloth_center += pm.position;
	}
	cloth_center /= cloth.point_masses.size();

	for (vector<int>::iterator it = mesh_inds.begin(); it != mesh_inds.end(); it++) {
		Mesh *m = (Mesh *) objects.at(*it);
		m->cloth_center = cloth_center;
	}

  // Initialize the ClothSimulator object
  app = new ClothSimulator(screen);
  app->loadCloth(&cloth);
  app->loadClothParameters(&cp);
  app->loadCollisionObjects(&objects);
  app->init();

  // Call this after all the widgets have been defined

  screen->setVisible(true);
  screen->performLayout();

  // Attach callbacks to the GLFW window

  setGLFWCallbacks();
	bool first = false;

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    glClearColor(0.25f, 0.25f, 0.25f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    app->drawContents();
		if (!first) {
			rtcCommitScene(scene);
			first = false;
		}

    // Draw nanogui
    screen->drawContents();
    screen->drawWidgets();

    glfwSwapBuffers(window);

    if (!app->isAlive()) {
      glfwSetWindowShouldClose(window, 1);
    }
  }
	
	rtcReleaseScene(scene);
	rtcReleaseDevice (device);

  return 0;
}
