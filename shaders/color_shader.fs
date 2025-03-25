#version 420 core
out vec4 FragColor;

layout (location = 0) in vec3 FragPos;

uniform vec4 color;

void main()
{
  FragColor = color;
}