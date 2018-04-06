#version 330
uniform sampler2D tex;
in vec2 frag_tex_coord;

out vec4 out_color;

void main() {
  out_color = texture(tex, frag_tex_coord);
}
