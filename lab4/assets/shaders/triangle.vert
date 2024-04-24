#version 410 core

layout(location = 0) in  vec2 a_Position;
layout(location = 1) in  vec3 a_Color;

layout(location = 0) out vec3 v_Color;

void main() {
    gl_Position = vec4(a_Position, 0., 1.);
    v_Color     = a_Color;
}
