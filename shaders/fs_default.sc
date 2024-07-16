$input v_position, v_normal, v_texcoord0

#include <bgfx_shader.sh>

uniform sampler2D material_texture_diffuse;
uniform sampler2D material_texture_specular;
uniform sampler2D material_emission;
uniform vec4      material_shininess; // float
uniform vec4      material_color;
uniform vec4      material_specular; // float

uniform vec4 dir_light_direction; // vec3
uniform vec4 dir_light_ambient;
uniform vec4 dir_light_diffuse;
uniform vec4 dir_light_specular;

#define MAX_NB_POINT_LIGHTS 20 
uniform vec4 point_light_position[MAX_NB_POINT_LIGHTS]; // vec3
uniform vec4 point_light_ambient[MAX_NB_POINT_LIGHTS];
uniform vec4 point_light_diffuse[MAX_NB_POINT_LIGHTS];
uniform vec4 point_light_specular[MAX_NB_POINT_LIGHTS];
uniform vec4 point_light_attenuation[MAX_NB_POINT_LIGHTS]; // vec3
uniform vec4 nb_point_light; // float

uniform vec4 view_position; // vec3

#define GAMMA 2.2f

vec4 CalcDirLight(vec3 light_direction, vec4 light_ambient,
                  vec4 light_diffuse, vec4 light_specular,
                  vec3 normal, vec3 view_dir, vec2 texcoord0);
vec4 CalcPointLight(vec3 light_position, vec4 light_ambient,
                    vec4 light_diffuse, vec4 light_specular,
                    vec3 light_attenuation,
                    vec3 frag_pose, vec3 normal, vec3 view_dir, vec2 texcoord0);

void main()
{
  // properties
  vec3 norm = normalize(v_normal);
  vec3 view_dir = normalize(view_position.xyz - v_position);

  // phase 1: Directional lighting
  vec4 result = CalcDirLight(dir_light_direction.xyz, dir_light_ambient,
                             dir_light_diffuse, dir_light_specular,
                             norm, view_dir, v_texcoord0);
  // phase 2: Point lights
  for(int i = 0; i < int(nb_point_light.x); i++)
      result += CalcPointLight(point_light_position[i].xyz, point_light_ambient[i],
                               point_light_diffuse[i], point_light_specular[i],
                               point_light_attenuation[i].xyz,
                               v_position, norm, view_dir, v_texcoord0);  

  gl_FragColor = result;
  gl_FragColor.rgb = pow(result.rgb, vec3(1.0f/GAMMA));
}

vec4 CalcDirLight(vec3 light_direction, vec4 light_ambient,
                  vec4 light_diffuse, vec4 light_specular,
                  vec3 normal, vec3 view_dir, vec2 texcoord0)
{
  vec3 light_dir = normalize(-light_direction.xyz);
  vec3 halfway_dir = normalize(light_dir + view_dir);
  // diffuse shading
  float diff = max(dot(normal, light_dir), 0.0);
  // specular shading
  float spec = pow(max(dot(view_dir, halfway_dir), 0.0), material_shininess.x);
  // combine results
  vec4 ambient  = light_ambient * texture2D(material_texture_diffuse, texcoord0) * material_color;
  vec4 diffuse  = light_diffuse  * diff * texture2D(material_texture_diffuse, texcoord0);
  vec4 specular = vec4(0.f);
  if(material_specular.x >= 0.f)
    specular = light_specular * spec * vec4(material_specular.x);
  else
    specular = light_specular * spec * vec4(texture2D(material_texture_specular, texcoord0).r);
  return (ambient + diffuse + specular);
}

vec4 CalcPointLight(vec3 light_position, vec4 light_ambient,
                    vec4 light_diffuse, vec4 light_specular,
                    vec3 light_attenuation,
                    vec3 frag_pose, vec3 normal, vec3 view_dir, vec2 texcoord0)
{
  vec3 light_dir = normalize(light_position - frag_pose);
  vec3 halfway_dir = normalize(light_dir + view_dir);
  // diffuse shading
  float diff = max(dot(normal, light_dir), 0.0);
  // specular shading
  float spec = pow(max(dot(view_dir, halfway_dir), 0.0), material_shininess.x);
  // attenuation
  float distance    = length(light_position - frag_pose);
  float attenuation = 1.0 / (light_attenuation.x + light_attenuation.y * distance + 
                      light_attenuation.z * (distance * distance));    
  // combine results
  vec4 ambient  = light_ambient  * texture2D(material_texture_diffuse, texcoord0);
  vec4 diffuse  = light_diffuse  * diff * texture2D(material_texture_diffuse, texcoord0);
  vec4 specular = vec4(0.f);
  if(material_specular.x >= 0.f)
    specular = light_specular * spec * vec4(material_specular.x);
  else
    specular = light_specular * spec * vec4(texture2D(material_texture_specular, texcoord0).r);

  ambient  *= attenuation;
  diffuse  *= attenuation;
  specular *= attenuation;
  return (ambient + diffuse + specular);
}