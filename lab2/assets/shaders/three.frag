#version 410 core

layout(location = 0) in  vec3 g_Position;
layout(location = 1) in  vec3 g_Normal;
layout(location = 2) in  vec2 g_TexCoord;

layout(location = 0) out vec4 f_Color;

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
    int   n       = 10;
    int   cb      = (int(g_TexCoord.x * n) + int(g_TexCoord.y * n)) % 2;
    vec3  cbCol   = cb * vec3(.8) + (1 - cb) * u_ObjectColor;
    vec3  texCol  = u_Wireframe || ! u_HasTexCoord ? u_ObjectColor : cbCol;
    float diffuse = max(dot(normalize(g_Normal), -u_LightDirection), 0.);
    float coeff   = u_Wireframe ? 1. : diffuse + u_Ambient;
    vec3  result  = coeff * u_LightColor * texCol;

    f_Color = vec4(pow(result, vec3(1. / 2.2)), 1.);
}
