$input v_position, v_normal, v_texcoord0

#include <bgfx_shader.sh>

SAMPLER2D(u_tex_color, 0);

uniform vec4 u_seg_color;

void main() {
    vec3 lightColor = vec3(1, 1, 1);
    vec3 lightPos = vec3(0, 0, 100);

    // ambient
    float ambientStrength = 0.1;
    vec3 ambient = ambientStrength * lightColor;

    // Diffuse
    vec3 norm = normalize(v_normal);
    vec3 lightDir = normalize(lightPos - v_position);

    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    vec4 result = vec4(ambient + diffuse, 1) * texture2D(u_tex_color, v_texcoord0) * u_seg_color;

	gl_FragColor = result;
}