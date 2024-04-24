#version 410 core

layout(location = 0) in  vec3 a_Position;
layout(location = 1) in  vec3 a_Normal;
layout(location = 2) in  vec2 a_TexCoord;

layout(location = 0) out vec3 v_Position;
layout(location = 1) out vec3 v_Normal;
layout(location = 2) out vec2 v_TexCoord;

layout(std140) uniform PassConstants {
    mat4  u_NormalTransform;
    mat4  u_Model;
    mat4  u_View;
    mat4  u_Projection;
    vec3  u_LightDirection;
    vec3  u_LightColor;
    vec3  u_ObjectColor;
    float u_Ambient;
    bool  u_HasTexCoord;
    bool  u_Wireframe;
    bool  u_Flat;
};

void main() {
    v_Position  = vec3(u_Model * vec4(a_Position, 1.));
    v_Normal    = vec3(u_NormalTransform * vec4(a_Normal, 1.));
    v_TexCoord  = a_TexCoord;
    gl_Position = u_Projection * u_View * vec4(v_Position, 1.);
}
