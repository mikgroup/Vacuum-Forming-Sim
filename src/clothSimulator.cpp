#include <cmath>
#include <glad/glad.h>

#include <CGL/vector3D.h>
#include "CGL/lodepng.h"
#include <nanogui/nanogui.h>
#include <sstream>

#include "clothSimulator.h"

#include "camera.h"
#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "collision/mesh.h"
#include "misc/camera_info.h"
#include "png.h"

using namespace nanogui;
using namespace std;

static GLuint makeTex(string path);

ClothSimulator::ClothSimulator(Screen *screen) {
  this->screen = screen;

  // Initialize OpenGL buffers and shaders

  wireframeShader.initFromFiles("Wireframe", "../shaders/camera.vert",
                                "../shaders/wireframe.frag");
  normalShader.initFromFiles("Normal", "../shaders/camera.vert",
                             "../shaders/normal.frag");
  phongShader.initFromFiles("Phong", "../shaders/camera.vert",
                            "../shaders/phong.frag");
	imageShader.initFromFiles("Image", "../shaders/camera.vert",
														"../shaders/image.frag");
	objectShader.initFromFiles("Object", "../shaders/camera.vert",
														"../shaders/object.frag");
	

  shaders.push_back(wireframeShader);
  shaders.push_back(normalShader);
  shaders.push_back(phongShader);
	shaders.push_back(imageShader);
  shaders.push_back(objectShader);

  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);
}

ClothSimulator::~ClothSimulator() {
  for (auto shader : shaders) {
    shader.free();
  }

  if (cloth) delete cloth;
  if (cp) delete cp;
  if (collision_objects) delete collision_objects;
}

void ClothSimulator::loadCloth(Cloth *cloth) { this->cloth = cloth; }

void ClothSimulator::loadClothParameters(ClothParameters *cp) { this->cp = cp; }

void ClothSimulator::loadCollisionObjects(vector<CollisionObject *> *objects) { this->collision_objects = objects; }

/**
 * Initializes the cloth simulation and spawns a new thread to separate
 * rendering from simulation.
 */
void ClothSimulator::init() {
  // Initialize GUI
  initGUI(screen);
  screen->setSize(default_window_size);

  // Initialize camera

  CGL::Collada::CameraInfo camera_info;
  camera_info.hFov = 50;
  camera_info.vFov = 35;
  camera_info.nClip = 0.01;
  camera_info.fClip = 10000;

  // Try to intelligently figure out the camera target

  Vector3D avg_pm_position(0, 0, 0);

  for (auto &pm : cloth->point_masses) {
    avg_pm_position += pm.position / cloth->point_masses.size();
  }
	
	textureId = makeTex(cloth->texture);
  cloth->write_to_file("cloth_original.off");
  cout << "Wrote original to file" << endl;

  CGL::Vector3D target(avg_pm_position.x, avg_pm_position.y / 2,
                       avg_pm_position.z);
  CGL::Vector3D c_dir(0., 0., 0.);
  canonical_view_distance = max(cloth->width, cloth->height) * 0.9;
  scroll_rate = canonical_view_distance / 10;

  view_distance = canonical_view_distance * 2;
  min_view_distance = canonical_view_distance / 10.0;
  max_view_distance = canonical_view_distance * 20.0;

  // canonicalCamera is a copy used for view resets

  camera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z), view_distance,
               min_view_distance, max_view_distance);
  canonicalCamera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z),
                        view_distance, min_view_distance, max_view_distance);

  screen_w = default_window_size(0);
  screen_h = default_window_size(1);

  camera.configure(camera_info, screen_w, screen_h);
  canonicalCamera.configure(camera_info, screen_w, screen_h);
}

bool ClothSimulator::isAlive() { return is_alive; }

void ClothSimulator::drawContents() {
  glEnable(GL_DEPTH_TEST);

  if (!is_paused) {
    vector<Vector3D> external_accelerations = {gravity};

    for (int i = 0; i < simulation_steps; i++) {
      cloth->simulate(frames_per_sec, simulation_steps, cp, external_accelerations, collision_objects);
  		for (CollisionObject *co : *collision_objects) {
   			co->update(frames_per_sec, simulation_steps);
  		}
    }
  }

  // Bind the active shader

  GLShader shader = shaders[activeShader];
  shader.bind();

  // Prepare the camera projection matrix

  Matrix4f model;
  model.setIdentity();

  Matrix4f view = getViewMatrix();
  Matrix4f projection = getProjectionMatrix();

  Matrix4f viewProjection = projection * view;

  shader.setUniform("model", model);
  shader.setUniform("viewProjection", viewProjection);

  switch (activeShader) {
  case WIREFRAME:
    drawWireframe(shader);
    break;
  case NORMALS:
    drawNormals(shader);
    break;
  case PHONG:
    drawPhong(shader);
    break;
	case IMAGE:
		drawImage(shader);
		shader = shaders[NORMALS]; // do not draw image on objects
		shader.bind();
		shader.setUniform("model", model);
		shader.setUniform("viewProjection", viewProjection);
		break;
  case OBJECT:
    drawObject(shader);
		shader = shaders[NORMALS]; // do not draw image on objects
		shader.bind();
		shader.setUniform("model", model);
		shader.setUniform("viewProjection", viewProjection);
    break;
  }

	if (!objects_hidden) {
  	for (CollisionObject *co : *collision_objects) {
   		co->render(shader);
  	}
	}
}

void ClothSimulator::drawWireframe(GLShader &shader) {
  int num_structural_springs =
      2 * cloth->num_width_points * cloth->num_height_points -
      cloth->num_width_points - cloth->num_height_points;
  int num_shear_springs =
      2 * (cloth->num_width_points - 1) * (cloth->num_height_points - 1);
  int num_bending_springs = num_structural_springs - cloth->num_width_points -
                            cloth->num_height_points;

  int num_springs = cp->enable_structural_constraints * num_structural_springs +
                    cp->enable_shearing_constraints * num_shear_springs +
                    cp->enable_bending_constraints * num_bending_springs;

  MatrixXf positions(3, num_springs * 2);
  MatrixXf normals(3, num_springs * 2);

	//printf("Expected Struct: %d, Shear %d, Bend %d\n", num_structural_springs, num_shear_springs, num_bending_springs);
  // Draw springs as lines

  int si = 0;

  for (int i = 0; i < cloth->springs.size(); i++) {
    Spring s = cloth->springs[i];

    if ((s.spring_type == STRUCTURAL && !cp->enable_structural_constraints) ||
        (s.spring_type == SHEARING && !cp->enable_shearing_constraints) ||
        (s.spring_type == BENDING && !cp->enable_bending_constraints)) {
      continue;
    }

    Vector3D pa = s.pm_a->position;
    Vector3D pb = s.pm_b->position;

    Vector3D na = s.pm_a->normal();
    Vector3D nb = s.pm_b->normal();

    positions.col(si) << pa.x, pa.y, pa.z;
    positions.col(si + 1) << pb.x, pb.y, pb.z;

    normals.col(si) << na.x, na.y, na.z;
    normals.col(si + 1) << nb.x, nb.y, nb.z;

    si += 2;
  }

  shader.setUniform("in_color", nanogui::Color(1.0f, 1.0f, 1.0f, 1.0f));
  shader.uploadAttrib("in_position", positions);
  shader.drawArray(GL_LINES, 0, num_springs * 2);
}

void ClothSimulator::drawNormals(GLShader &shader) {
  int num_tris = cloth->clothMesh->triangles.size();

  MatrixXf positions(3, num_tris * 3);
  MatrixXf normals(3, num_tris * 3);

  for (int i = 0; i < num_tris; i++) {
    Triangle *tri = cloth->clothMesh->triangles[i];

    Vector3D p1 = tri->pm1->position;
    Vector3D p2 = tri->pm2->position;
    Vector3D p3 = tri->pm3->position;

    Vector3D n1 = tri->pm1->normal();
    Vector3D n2 = tri->pm2->normal();
    Vector3D n3 = tri->pm3->normal();

    positions.col(i * 3) << p1.x, p1.y, p1.z;
    positions.col(i * 3 + 1) << p2.x, p2.y, p2.z;
    positions.col(i * 3 + 2) << p3.x, p3.y, p3.z;

    normals.col(i * 3) << n1.x, n1.y, n1.z;
    normals.col(i * 3 + 1) << n2.x, n2.y, n2.z;
    normals.col(i * 3 + 2) << n3.x, n3.y, n3.z;
  }

  shader.uploadAttrib("in_position", positions);
  shader.uploadAttrib("in_normal", normals);

  shader.drawArray(GL_TRIANGLES, 0, num_tris * 3);
}


static double u3, v3;
static GLuint makeTex(string path) {
	std::vector<unsigned char> image;
	unsigned width, height;
  int error = lodepng::decode(image, width, height, path);
  if (error != 0) {
		std::cout << "error " << error << ": " << lodepng_error_text(error) << std::endl;
    return 1;
	}	

	// Texture size must be power of two for the primitive OpenGL version this is written for. Find next power of two.
  size_t u2 = 1; while(u2 < width) u2 *= 2;
  size_t v2 = 1; while(v2 < height) v2 *= 2;
  // Ratio for power of two version compared to actual version, to render the non power of two image with proper size.
  /*double*/ u3 = (double)width / u2;
  /*double*/ v3 = (double)height / v2;
    std::cout << u3 << std::endl;
    std::cout << v3 << std::endl;
	// Make power of 2 version of the image.
	std::vector<unsigned char> image2(u2 * v2 * 4);
  for(size_t y = 0; y < height; y++)
    for(size_t x = 0; x < width; x++)
        for(size_t c = 0; c < 4; c++) 
            image2[4 * u2 * y + 4 * x + c] = image[4 * width * y + 4 * x + c]; 

  GLuint textureID;
	
  glGenTextures(1, &textureID); // generate 1 texture id
  
	glBindTexture(GL_TEXTURE_2D, textureID);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);


  //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, u2, v2, 0, GL_RGBA, GL_UNSIGNED_BYTE, &image2[0]);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, &image[0]);
  
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  return textureID;
}

void ClothSimulator::drawImage(GLShader &shader) {
  int num_tris = cloth->clothMesh->triangles.size();

  MatrixXf positions(3, num_tris * 3);
  MatrixXf normals(3, num_tris * 3);
  MatrixXf uvs(2, num_tris * 3);


  for (int i = 0; i < num_tris; i++) {
    Triangle *tri = cloth->clothMesh->triangles[i];

    Vector3D p1 = tri->pm1->position;
    Vector3D p2 = tri->pm2->position;
    Vector3D p3 = tri->pm3->position;

    Vector3D n1 = tri->pm1->normal();
    Vector3D n2 = tri->pm2->normal();
    Vector3D n3 = tri->pm3->normal();

    positions.col(i * 3) << p1.x, p1.y, p1.z;
    positions.col(i * 3 + 1) << p2.x, p2.y, p2.z;
    positions.col(i * 3 + 2) << p3.x, p3.y, p3.z;

    normals.col(i * 3) << n1.x, n1.y, n1.z;
    normals.col(i * 3 + 1) << n2.x, n2.y, n2.z;
    normals.col(i * 3 + 2) << n3.x, n3.y, n3.z;

		//uvs.col(i * 3) 		 <<	tri->pm1->uv.x - floor(tri->pm1->uv.x), tri->pm1->uv.y - floor(tri->pm1->uv.y);
		//uvs.col(i * 3 + 1) << tri->pm2->uv.x - floor(tri->pm2->uv.x), tri->pm2->uv.y - floor(tri->pm2->uv.y);
		//uvs.col(i * 3 + 2) << tri->pm3->uv.x - floor(tri->pm3->uv.x), tri->pm3->uv.y - floor(tri->pm3->uv.y);
		
		uvs.col(i * 3)     << tri->pm1->uv.x, tri->pm1->uv.y;
		uvs.col(i * 3 + 1) << tri->pm2->uv.x, tri->pm2->uv.y;
		uvs.col(i * 3 + 2) << tri->pm3->uv.x, tri->pm3->uv.y;
  }

  Vector3D cp = camera.position();

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureId);
	//shader.setUniform("tex", 0);

  shader.uploadAttrib("in_position", positions);
  shader.uploadAttrib("in_normal", normals);
	shader.uploadAttrib("vert_tex_coord", uvs);
  
  shader.setUniform("eye", Vector3f(cp.x, cp.y, cp.z));
  //shader.setUniform("light", Vector3f(0.5, 2, 2));

  shader.drawArray(GL_TRIANGLES, 0, num_tris * 3);
}

void ClothSimulator::drawObject(GLShader &shader) {
  int num_tris = cloth->clothMesh->triangles.size();

  MatrixXf positions(3, num_tris * 3);
  MatrixXf normals(3, num_tris * 3);
  MatrixXf colors(4, num_tris * 3); 

  for (int i = 0; i < num_tris; i++) {
    Triangle *tri = cloth->clothMesh->triangles[i];
    
    PointMass *p[3];
    p[0] = tri->pm1;
    p[1] = tri->pm2;
    p[2] = tri->pm3;

    Vector3D p1 = tri->pm1->position;
    Vector3D p2 = tri->pm2->position;
    Vector3D p3 = tri->pm3->position;

    Vector3D n1 = tri->pm1->normal();
    Vector3D n2 = tri->pm2->normal();
    Vector3D n3 = tri->pm3->normal();

    positions.col(i * 3) << p1.x, p1.y, p1.z;
    positions.col(i * 3 + 1) << p2.x, p2.y, p2.z;
    positions.col(i * 3 + 2) << p3.x, p3.y, p3.z;

    normals.col(i * 3) << n1.x, n1.y, n1.z;
    normals.col(i * 3 + 1) << n2.x, n2.y, n2.z;
    normals.col(i * 3 + 2) << n3.x, n3.y, n3.z;
    
    nanogui::Color p_color = color;
    nanogui::Color rgb[3];
    rgb[0] = nanogui::Color(255,0,0,1);
    rgb[1] = nanogui::Color(0,255,0,1);
    rgb[2] = nanogui::Color(0,0,255,1);
    
    int r = 0, g = 0, b = 0;
    for (int j = 0; j < 3; j++) {
      //if (p[j]->stuck) {
      if (p[j]->collide_id != -1) {
        p_color = rgb[p[j]->collide_id]; 
      }
      
      colors.col(i * 3 + j) << p_color.r(), p_color.g(), p_color.b(), 1;
      //colors.col(i * 3 + j) << 255,0,0,1;
    }
  }

  Vector3D cp = camera.position();

  shader.setUniform("eye", Vector3f(cp.x, cp.y, cp.z));
  //shader.setUniform("light", Vector3f(0.5, 2, 2));

  shader.uploadAttrib("in_position", positions);
  shader.uploadAttrib("in_normal", normals);
  shader.uploadAttrib("in_color", colors);

  shader.drawArray(GL_TRIANGLES, 0, num_tris * 3);

}


void ClothSimulator::drawPhong(GLShader &shader) {
  int num_tris = cloth->clothMesh->triangles.size();

  MatrixXf positions(3, num_tris * 3);
  MatrixXf normals(3, num_tris * 3);

  for (int i = 0; i < num_tris; i++) {
    Triangle *tri = cloth->clothMesh->triangles[i];

    Vector3D p1 = tri->pm1->position;
    Vector3D p2 = tri->pm2->position;
    Vector3D p3 = tri->pm3->position;

    Vector3D n1 = tri->pm1->normal();
    Vector3D n2 = tri->pm2->normal();
    Vector3D n3 = tri->pm3->normal();

    positions.col(i * 3) << p1.x, p1.y, p1.z;
    positions.col(i * 3 + 1) << p2.x, p2.y, p2.z;
    positions.col(i * 3 + 2) << p3.x, p3.y, p3.z;

    normals.col(i * 3) << n1.x, n1.y, n1.z;
    normals.col(i * 3 + 1) << n2.x, n2.y, n2.z;
    normals.col(i * 3 + 2) << n3.x, n3.y, n3.z;
  }

  Vector3D cp = camera.position();

  shader.setUniform("in_color", color);

  shader.setUniform("eye", Vector3f(cp.x, cp.y, cp.z));
  //shader.setUniform("light", Vector3f(0.5, 2, 2));

  shader.uploadAttrib("in_position", positions);
  shader.uploadAttrib("in_normal", normals);

  shader.drawArray(GL_TRIANGLES, 0, num_tris * 3);

}

// ----------------------------------------------------------------------------
// CAMERA CALCULATIONS
//
// OpenGL 3.1 deprecated the fixed pipeline, so we lose a lot of useful OpenGL
// functions that have to be recreated here.
// ----------------------------------------------------------------------------

void ClothSimulator::resetCamera() { camera.copy_placement(canonicalCamera); }

Matrix4f ClothSimulator::getProjectionMatrix() {
  Matrix4f perspective;
  perspective.setZero();

  double near = camera.near_clip();
  double far = camera.far_clip();

  double theta = camera.v_fov() * M_PI / 360;
  double range = far - near;
  double invtan = 1. / tanf(theta);

  perspective(0, 0) = invtan / camera.aspect_ratio();
  perspective(1, 1) = invtan;
  perspective(2, 2) = -(near + far) / range;
  perspective(3, 2) = -1;
  perspective(2, 3) = -2 * near * far / range;
  perspective(3, 3) = 0;

  return perspective;
}

Matrix4f ClothSimulator::getViewMatrix() {
  Matrix4f lookAt;
  Matrix3f R;

  lookAt.setZero();

  // Convert CGL vectors to Eigen vectors
  // TODO: Find a better way to do this!

  CGL::Vector3D c_pos = camera.position();
  CGL::Vector3D c_udir = camera.up_dir();
  CGL::Vector3D c_target = camera.view_point();

  Vector3f eye(c_pos.x, c_pos.y, c_pos.z);
  Vector3f up(c_udir.x, c_udir.y, c_udir.z);
  Vector3f target(c_target.x, c_target.y, c_target.z);

  R.col(2) = (eye - target).normalized();
  R.col(0) = up.cross(R.col(2)).normalized();
  R.col(1) = R.col(2).cross(R.col(0));

  lookAt.topLeftCorner<3, 3>() = R.transpose();
  lookAt.topRightCorner<3, 1>() = -R.transpose() * eye;
  lookAt(3, 3) = 1.0f;

  return lookAt;
}

// ----------------------------------------------------------------------------
// EVENT HANDLING
// ----------------------------------------------------------------------------

bool ClothSimulator::cursorPosCallbackEvent(double x, double y) {
  if (left_down && !middle_down && !right_down) {
    if (ctrl_down) {
      mouseRightDragged(x, y);
    } else {
      mouseLeftDragged(x, y);
    }
  } else if (!left_down && !middle_down && right_down) {
    mouseRightDragged(x, y);
  } else if (!left_down && !middle_down && !right_down) {
    mouseMoved(x, y);
  }

  mouse_x = x;
  mouse_y = y;

  return true;
}

bool ClothSimulator::mouseButtonCallbackEvent(int button, int action,
                                              int modifiers) {
  switch (action) {
  case GLFW_PRESS:
    switch (button) {
    case GLFW_MOUSE_BUTTON_LEFT:
      left_down = true;
      break;
    case GLFW_MOUSE_BUTTON_MIDDLE:
      middle_down = true;
      break;
    case GLFW_MOUSE_BUTTON_RIGHT:
      right_down = true;
      break;
    }
    return true;

  case GLFW_RELEASE:
    switch (button) {
    case GLFW_MOUSE_BUTTON_LEFT:
      left_down = false;
      break;
    case GLFW_MOUSE_BUTTON_MIDDLE:
      middle_down = false;
      break;
    case GLFW_MOUSE_BUTTON_RIGHT:
      right_down = false;
      break;
    }
    return true;
  }

  return false;
}

void ClothSimulator::mouseMoved(double x, double y) { y = screen_h - y; }

void ClothSimulator::mouseLeftDragged(double x, double y) {
  float dx = x - mouse_x;
  float dy = y - mouse_y;

  camera.rotate_by(-dy * (PI / screen_h), -dx * (PI / screen_w));
}

void ClothSimulator::mouseRightDragged(double x, double y) {
  camera.move_by(mouse_x - x, y - mouse_y, canonical_view_distance);
}

bool ClothSimulator::keyCallbackEvent(int key, int scancode, int action,
                                      int mods) {
  ctrl_down = (bool)(mods & GLFW_MOD_CONTROL);

  if (action == GLFW_PRESS) {
    switch (key) {
    case GLFW_KEY_ESCAPE:
      is_alive = false;
      break;
    case 'r':
    case 'R':
      cloth->reset();
      break;
    case ' ':
      resetCamera();
      break;
    case 'p':
    case 'P':
      is_paused = !is_paused;
      break;
    case 'n':
    case 'N':
      if (is_paused) {
        is_paused = false;
        drawContents();
        is_paused = true;
      }
      break; 
		case 'h':
		case 'H':
			objects_hidden = !objects_hidden;
			break;
		case 'w':
		case 'W': // Write to file
			cloth->write_to_file("cloth.off");
			system("./arap cloth.off");
			cloth->remap_uvs();
			break;
    case 'o':
    case 'O': // Write to obj
      cloth->write_obj("cloth.obj");
      break;
    case 's':
    case 'S':
      // Figure out needed dimensions
      // Rescale width and height to a fixed standard to give proj1
      double newWidth = pngWidth;
      double rescale = newWidth / cloth->width;
      double newHeight = cloth->height * rescale;

      // Cast to strings
      std::ostringstream w;
      w << newWidth;
      std::ostringstream h;
      h << newHeight;

      // Write to svg
      string svgFile = "test_svg.svg";
      cloth->write_to_svg(svgFile, "../storage/tex.png", newWidth, newHeight, rescale);
      cout << "Saved svg as " << svgFile << ".\n";
      cout << "Rasterizing to png...\n";
      system(("../ext/proj1/build/draw " + svgFile + " " + h.str() + " " + w.str()).c_str());
      cout << "PNG saved to test.png.\n";
      break;
		}
  }

  return true;
}

bool ClothSimulator::dropCallbackEvent(int count, const char **filenames) {
  return true;
}

bool ClothSimulator::scrollCallbackEvent(double x, double y) {
  camera.move_forward(y * scroll_rate);
  return true;
}

bool ClothSimulator::resizeCallbackEvent(int width, int height) {
  screen_w = width;
  screen_h = height;

  camera.set_screen_size(screen_w, screen_h);
  return true;
}

void ClothSimulator::initGUI(Screen *screen) {
  Window *window = new Window(screen, "Settings");
  window->setPosition(Vector2i(15, 15));
  window->setLayout(new GroupLayout(15, 6, 14, 5));

  // Spring types

  new Label(window, "Spring types", "sans-bold");

  {
    Button *b = new Button(window, "structural");
    b->setFlags(Button::ToggleButton);
    b->setPushed(cp->enable_structural_constraints);
    b->setFontSize(14);
    b->setChangeCallback(
        [this](bool state) { cp->enable_structural_constraints = state; });

    b = new Button(window, "shearing");
    b->setFlags(Button::ToggleButton);
    b->setPushed(cp->enable_shearing_constraints);
    b->setFontSize(14);
    b->setChangeCallback(
        [this](bool state) { cp->enable_shearing_constraints = state; });

    b = new Button(window, "bending");
    b->setFlags(Button::ToggleButton);
    b->setPushed(cp->enable_bending_constraints);
    b->setFontSize(14);
    b->setChangeCallback(
        [this](bool state) { cp->enable_bending_constraints = state; });
  }

  // Mass-spring parameters

  new Label(window, "Parameters", "sans-bold");

  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);

    new Label(panel, "density :", "sans-bold");

    FloatBox<double> *fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(cp->density / 10);
    fb->setUnits("g/cm^2");
    fb->setSpinnable(true);
    fb->setCallback([this](float value) { cp->density = (double)(value * 10); });

    new Label(panel, "ks :", "sans-bold");

    fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(cp->ks);
    fb->setUnits("N/m");
    fb->setSpinnable(true);
    fb->setMinValue(0);
    fb->setCallback([this](float value) { cp->ks = value; });
  }

  // Simulation constants

  new Label(window, "Simulation", "sans-bold");

  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);

    new Label(panel, "frames/s :", "sans-bold");

    IntBox<int> *fsec = new IntBox<int>(panel);
    fsec->setEditable(true);
    fsec->setFixedSize(Vector2i(100, 20));
    fsec->setFontSize(14);
    fsec->setValue(frames_per_sec);
    fsec->setSpinnable(true);
    fsec->setCallback([this](int value) { frames_per_sec = value; });

    new Label(panel, "steps/frame :", "sans-bold");

    IntBox<int> *num_steps = new IntBox<int>(panel);
    num_steps->setEditable(true);
    num_steps->setFixedSize(Vector2i(100, 20));
    num_steps->setFontSize(14);
    num_steps->setValue(simulation_steps);
    num_steps->setSpinnable(true);
    num_steps->setMinValue(0);
    num_steps->setCallback([this](int value) { simulation_steps = value; });
  }

  // Damping slider and textbox

  new Label(window, "Damping", "sans-bold");

  {
    Widget *panel = new Widget(window);
    panel->setLayout(
        new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));

    Slider *slider = new Slider(panel);
    slider->setValue(cp->damping);
    slider->setFixedWidth(105);

    TextBox *percentage = new TextBox(panel);
    percentage->setFixedWidth(75);
    percentage->setValue(to_string(cp->damping));
    percentage->setUnits("%");
    percentage->setFontSize(14);

    slider->setCallback([percentage](float value) {
      percentage->setValue(std::to_string(value));
    });
    slider->setFinalCallback([&](float value) {
      cp->damping = (double)value;
      // cout << "Final slider value: " << (int)(value * 100) << endl;
    });
  }

  // Gravity

  new Label(window, "Gravity", "sans-bold");

  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);

    new Label(panel, "x :", "sans-bold");

    FloatBox<double> *fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(gravity.x);
    fb->setUnits("m/s^2");
    fb->setSpinnable(true);
    fb->setCallback([this](float value) { gravity.x = value; });

    new Label(panel, "y :", "sans-bold");

    fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(gravity.y);
    fb->setUnits("m/s^2");
    fb->setSpinnable(true);
    fb->setCallback([this](float value) { gravity.y = value; });

    new Label(panel, "z :", "sans-bold");

    fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(gravity.z);
    fb->setUnits("m/s^2");
    fb->setSpinnable(true);
    fb->setCallback([this](float value) { gravity.z = value; });
  }

  // Appearance

  new Label(window, "Appearance", "sans-bold");

  {
    ComboBox *cb = new ComboBox(window, {"Wireframe", "Normals", "Shaded", "Image", "Objects"});
    cb->setFontSize(14);
		cb->setSelectedIndex(activeShader);
    cb->setCallback(
        [this, screen](int idx) { activeShader = static_cast<e_shader>(idx); });

		Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);
				
		Slider *slider_trans_x = new Slider(panel);
		slider_trans_x->setValue(0.5);
		slider_trans_x->setFixedWidth(105);

		TextBox *value_x = new TextBox(panel);
		value_x->setFixedWidth(75);
		value_x->setValue(to_string(0));
		value_x->setFontSize(14);

		slider_trans_x->setCallback([value_x](float val) {
			value_x->setValue(std::to_string(val - 0.5));
		});
		slider_trans_x->setFinalCallback([&](float value) {
			cloth->translate_uvs((value - 0.5)/4, 0);
		});


		Slider *slider_trans_y = new Slider(panel);
		slider_trans_y->setValue(0.5);
		slider_trans_y->setFixedWidth(105);

		TextBox *value_y = new TextBox(panel);
		value_y->setFixedWidth(75);
		value_y->setValue(to_string(0));
		value_y->setFontSize(14);

		slider_trans_y->setCallback([value_y](float val) {
			value_y->setValue(std::to_string(val - 0.5));
		});
		slider_trans_y->setFinalCallback([&](float value) {
			cloth->translate_uvs(0, (value - 0.5)/4);
		});
		
		Slider *slider_scale = new Slider(panel);
		slider_scale->setValue(0.5);
		slider_scale->setFixedWidth(105);

		TextBox *value_scale = new TextBox(panel);
		value_scale->setFixedWidth(75);
		value_scale->setValue(to_string(0.5));
		value_scale->setFontSize(14);

		slider_scale->setCallback([value_scale](float val) {
			value_scale->setValue(std::to_string(val * 2));
		});
		slider_scale->setFinalCallback([&](float value) {
			cloth->scale_uvs(2 - value * 2.0);
		});

		Slider *slider_rotate = new Slider(panel);
		slider_rotate->setValue(0.5);
		slider_rotate->setFixedWidth(105);

		TextBox *value_rotate = new TextBox(panel);
		value_rotate->setFixedWidth(75);
		value_rotate->setValue(to_string(0));
		value_rotate->setFontSize(14);

		slider_rotate->setCallback([value_rotate](float val) {
			value_rotate->setValue(std::to_string((val - 0.5) * M_PI / 8));
		});
		slider_rotate->setFinalCallback([&](float value) {
			cloth->rotate_uvs((value - 0.5) * M_PI / 8);
		});

    new Label(panel, "PNG Height :", "sans-bold");

    FloatBox<double> *fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(512);
    fb->setUnits("px");
    fb->setSpinnable(true);
    fb->setCallback([this](float value) { pngWidth = value; });
  }

  new Label(window, "Color", "sans-bold");

  {
    ColorWheel *cw = new ColorWheel(window, color);
    cw->setCallback(
        [this](const nanogui::Color &color) { this->color = color; });
  }
}
