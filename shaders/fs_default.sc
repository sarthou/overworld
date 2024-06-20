$input v_color0, v_texcoord0

#include <bgfx_shader.sh>

SAMPLER2D(u_tex_color, 0);

uniform vec4 u_seg_color;

void main() {
    vec4 t_color = texture2D(u_tex_color, v_texcoord0);
	gl_FragColor = t_color * u_seg_color; // v_color0
}