#version 410 core

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

layout(location = 0) in  vec3 v_Position[];
layout(location = 1) in  vec3 v_Normal[];
layout(location = 2) in  vec2 v_TexCoord[];

layout(location = 0) out vec3 g_Position;
layout(location = 1) out vec3 g_Normal;
layout(location = 2) out vec2 g_TexCoord;

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
    vec3  normal = normalize(cross(v_Position[1] - v_Position[0], v_Position[2] - v_Position[0]));

    gl_Position = gl_in[0].gl_Position;
    g_Position  = v_Position[0];
    g_Normal    = u_Flat ? normal : v_Normal[0];
    g_TexCoord  = v_TexCoord[0];
    EmitVertex();

    gl_Position = gl_in[1].gl_Position;
    g_Position  = v_Position[1];
    g_Normal    = u_Flat ? normal : v_Normal[1];
    g_TexCoord  = v_TexCoord[1];
    EmitVertex();
    
    gl_Position = gl_in[2].gl_Position;
    g_Position  = v_Position[2];
    g_Normal    = u_Flat ? normal : v_Normal[2];
    g_TexCoord  = v_TexCoord[2];
    EmitVertex();

    EndPrimitive();
}
