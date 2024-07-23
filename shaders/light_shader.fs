#version 420 core
out vec4 FragColor;

layout (location = 0) in vec3 FragPos;
layout (location = 1) in vec3 Normal;
layout (location = 2) in vec2 TexCoords;

struct Material {
  sampler2D texture_diffuse1;
  sampler2D texture_specular1;
  sampler2D emission;
  float     shininess;
  float     specular;
  vec4      color;
}; 

uniform Material material;

struct DirLight {
  vec4 direction;

  vec4 ambient;
  vec4 diffuse;
  vec4 specular;
};  
uniform DirLight dir_light; 

struct PointLight {    
  vec4 position;

  vec4 ambient;
  vec4 diffuse;
  vec4 specular;
  vec4 attenuation;
};  
#define NR_POINT_LIGHTS 20 
uniform PointLight point_lights[NR_POINT_LIGHTS];
uniform float nb_point_lights;

uniform vec3 view_pose;

uniform mat4 view;

layout (std140) uniform LightSpaceMatrices
{
    mat4 light_space_matrices[16];
};
uniform sampler2DArray shadow_maps;
uniform float cascade_planes_distances[16];
uniform int cascade_count;
uniform float far_plane;

vec4 calcDirLight(DirLight light, vec3 normal, vec3 viewDir);
vec4 calcPointLight(PointLight light, vec3 normal, vec3 viewDir);
float ShadowCalculation(vec3 fragPosWorldSpace);


void main()
{
  // properties
  vec3 norm = normalize(Normal);
  vec3 viewDir = normalize(view_pose - FragPos);

  // phase 1: Directional lighting
  vec4 result = calcDirLight(dir_light, norm, viewDir);
  
  // phase 2: Point lights
  for(int i = 0; i < nb_point_lights; i++)
      result += calcPointLight(point_lights[i], norm, viewDir);    
  
  FragColor = result;
  float gamma = 2.2;
  FragColor.rgb = pow(FragColor.rgb, vec3(1.0/gamma));
}

vec4 calcDirLight(DirLight light, vec3 normal, vec3 viewDir)
{
  vec3 lightDir = normalize(-light.direction.xyz);
  vec3 halfwayDir = normalize(lightDir + viewDir);
  // diffuse shading
  float diff = max(dot(normal, lightDir), 0.0);
  // specular shading
  //float spec = pow(max(dot(normal, halfwayDir), 0.0), material.shininess);
  vec3 reflectDir = reflect(-lightDir, normal);
  float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
  // combine results
  vec4 mat_ambient = material.color / 255.F;
  if(material.color.w == 0)
    mat_ambient = texture(material.texture_diffuse1, TexCoords);
  vec4 mat_spec = vec4(material.specular);
  if(material.specular < 0)
    mat_spec = vec4(texture(material.texture_specular1, TexCoords).r);

  float shadow = ShadowCalculation(FragPos);

  vec4 ambient  = light.ambient  * mat_ambient;
  vec4 diffuse  = light.diffuse  * diff * mat_ambient;
  vec4 specular = light.specular * spec * mat_spec;
  return (ambient + (1.0 - shadow) * (diffuse + specular));
}

vec4 calcPointLight(PointLight light, vec3 normal, vec3 viewDir)
{
  vec3 lightDir = normalize(light.position.xyz - FragPos);
  vec3 halfwayDir = normalize(lightDir + viewDir);
  // diffuse shading
  float diff = max(dot(normal, lightDir), 0.0);
  // specular shading
  //float spec = pow(max(dot(normal, halfwayDir), 0.0), material.shininess);
  vec3 reflectDir = reflect(-lightDir, normal);
  float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
  // attenuation
  float distance    = length(vec3(light.position.xyz) - FragPos);
  float attenuation = 1.0 / (light.attenuation.x + light.attenuation.y * distance + 
                      light.attenuation.z * (distance * distance));    
  // combine results
  vec4 mat_ambient = material.color / 255.f;
  if(material.color.w == 0)
    mat_ambient = texture(material.texture_diffuse1, TexCoords);
  vec4 mat_spec = vec4(material.specular);
  if(material.specular <= 0)
    mat_spec = vec4(texture(material.texture_specular1, TexCoords).r);

  vec4 ambient  = light.ambient  * mat_ambient;
  vec4 diffuse  = light.diffuse  * diff * mat_ambient;
  vec4 specular = light.specular * spec * mat_spec;
  ambient  *= attenuation;
  diffuse  *= attenuation;
  specular *= attenuation;
  return (ambient + diffuse + specular);
}

float ShadowCalculation(vec3 fragPosWorldSpace)
{
  // select cascade layer
  vec4 frag_pose_view_space = view * vec4(fragPosWorldSpace, 1.0);
  float depth_value = abs(frag_pose_view_space.z);

  int layer = -1;
  for (int i = 0; i < cascade_count; ++i)
  {
    if (depth_value < cascade_planes_distances[i])
    {
      layer = i;
      break;
    }
  }

  if (layer == -1)
  {
    layer = cascade_count;
  }

  //return layer;

  vec4 frag_pos_light_space = light_space_matrices[layer] * vec4(fragPosWorldSpace, 1.0);
  // perform perspective divide
  vec3 proj_coords = frag_pos_light_space.xyz / frag_pos_light_space.w;
  // transform to [0,1] range
  proj_coords = proj_coords * 0.5 + 0.5;

  // get depth of current fragment from light's perspective
  float current_depth = proj_coords.z;

  // keep the shadow at 0.0 when outside the far_plane region of the light's frustum.
  if (current_depth > 1.0)
  {
    return 0.0;
  }

  // calculate bias (based on depth map resolution and slope)
  vec3 normal = normalize(Normal);
  float bias = max(0.05 * (1.0 - dot(normal, dir_light.direction.xyz)), 0.005);
  const float bias_modifier = 7.; // Can be tuned (increase reduce near to wall missing shadow)
                                  // Increasing it too much will reduce the far quality 
  if (layer == cascade_count)
  {
    bias *= 1 / (far_plane * bias_modifier);
  }
  else
  {
    bias *= 1 / (cascade_planes_distances[layer] * bias_modifier);
  }

  // PCF
  float shadow = 0.0;
  vec2 texel_size = 1.0 / vec2(textureSize(shadow_maps, 0));
  for(int x = -1; x <= 1; ++x)
  {
    for(int y = -1; y <= 1; ++y)
    {
      float pcf_depth = texture(shadow_maps, vec3(proj_coords.xy + vec2(x, y) * texel_size, layer)).r;
      shadow += (current_depth - bias) > pcf_depth ? 1.0 : 0.0;        
    }    
  }
  shadow /= 9.0;
      
  return shadow;
}