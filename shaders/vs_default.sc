$input a_position, a_normal, a_texcoord0
$output v_position, v_normal, v_texcoord0

#include <bgfx_shader.sh>

void main() {
    vec3 light_position = vec3(0, 0, 100);

	gl_Position = mul(u_modelViewProj, vec4(a_position, 1.0));
	v_position = vec3(u_model[0] * vec4(a_position, 1.0));
	v_normal = a_normal;
	v_texcoord0 = a_texcoord0;
}