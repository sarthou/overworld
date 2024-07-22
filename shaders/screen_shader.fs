#version 330 core
out vec4 FragColor;

in vec2 TexCoord;

uniform sampler2D screenTexture;
  
void main()
{
  FragColor = vec4(texture(screenTexture, TexCoord).rgb, 1.0);
}