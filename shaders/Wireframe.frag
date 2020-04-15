#version 330

uniform vec4 u_color;

in vec4 v_position;
in vec4 v_normal;

out vec4 out_color;

void main() {
  out_color = u_color;
}
