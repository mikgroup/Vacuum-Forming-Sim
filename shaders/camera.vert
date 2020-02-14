#version 330

uniform mat4 model;
uniform mat4 viewProjection;

in vec4 in_position;
in vec4 in_normal;
in vec4 in_color;
in vec2 vert_tex_coord;

out vec4 vertex;
out vec4 normal;
out vec2 frag_tex_coord;
out vec4 object_color;

void main() {
  gl_PointSize = 4.0;
  gl_Position = viewProjection * model * in_position;

  vertex = in_position;
  normal = in_normal;
	frag_tex_coord = vert_tex_coord;
  object_color = in_color;
}
