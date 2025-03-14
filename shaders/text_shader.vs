#version 330 core

layout (location = 0) in vec3 vertex; // <vec2 pos, vec2 tex>
layout (location = 1) in vec2 textcoord; // <vec2 pos, vec2 tex>

out vec2 TexCoords;

uniform mat4 projection;

void main()
{
  gl_Position = projection * vec4(vertex.x, vertex.y, vertex.z, 1.0);
  TexCoords = textcoord;
}  