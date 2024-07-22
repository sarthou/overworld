#version 420 core
out vec4 FragColor;

layout (location = 0) in vec3 FragPos;
layout (location = 1) in vec3 Normal;  
layout (location = 2) in vec2 TexCoords;

struct Material {
  sampler2D texture_diffuse1;
  sampler2D texture_diffuse2;
  sampler2D texture_diffuse3;
  sampler2D texture_specular1;
  sampler2D texture_specular2;
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

vec4 calcDirLight(DirLight light, vec3 normal, vec3 viewDir);
vec4 calcPointLight(PointLight light, vec3 normal, vec3 viewDir);

float near = 0.1; 
float far  = 100.0; 
float linearizeDepth(float depth) 
{
  float z = depth * 2.0 - 1.0; // back to NDC 
  return (2.0 * near * far) / (far + near - z * (far - near));	
}

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
  //float gamma = 2.2;
  //FragColor.rgb = pow(FragColor.rgb, vec3(1.0/gamma));

  // float depth = linearizeDepth(gl_FragCoord.z) / far; // divide by far for demonstration
  // FragColor = vec4(vec3(depth), 1.0);
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

  vec4 ambient  = light.ambient  * mat_ambient;
  vec4 diffuse  = light.diffuse  * diff * mat_ambient;
  vec4 specular = light.specular * spec * mat_spec;
  return (ambient + diffuse + specular);
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