$input v_position

#include <bgfx_shader.sh>

uniform samplerCube cube_texture;

void main()
{
  gl_FragColor = textureCube(cube_texture, v_position);;
}
