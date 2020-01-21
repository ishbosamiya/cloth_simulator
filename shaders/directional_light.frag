#version 330 core
out vec4 FragColor;

struct Material {
  vec3 color;
  vec3 specular;
  float shininess;
};

struct Light {
  vec3 direction;

  vec3 ambient;
  vec3 diffuse;
  vec3 specular;
};

in vec2 TexCoord;
in vec3 Normal;
in vec3 FragPos;

uniform vec3 viewPos;
uniform Material material;
uniform Light light;

void main()
{
  vec3 ambient = light.ambient * material.color;

  vec3 norm = normalize(Normal);
  vec3 light_dir = normalize(-light.direction);
  float diff = max(dot(norm, light_dir), 0.0);
  vec3 diffuse = light.diffuse * diff * material.color;

  vec3 view_dir = normalize(viewPos - FragPos);
  vec3 reflect_dir = reflect(-light_dir, norm);
  float spec = pow(max(dot(view_dir, reflect_dir), 0.0), material.shininess);
  vec3 specular = light.specular * spec * material.specular;

  vec3 result = ambient + diffuse + specular;
  FragColor = vec4(result, 1.0);
}
