#version 330 core
out vec4 FragColor;

in vec3 TexCoord;
in vec3 Normal;
in vec3 FragPos;

void main()
{
  vec3 light_color = vec3(1.0, 1.0, 1.0);
  vec3 object_color = vec3(1.0, 0.5, 0.31);
  vec3 light_pos = vec3(1.2, 3.0, 3.0);

  float ambient_strength = 0.1;
  vec3 ambient = ambient_strength * light_color;

  vec3 norm = normalize(Normal);
  vec3 light_dir = normalize(light_pos - FragPos);
  float diff = max(dot(norm, light_dir), 0.0);
  vec3 diffuse = diff * light_color;

  vec3 result = (ambient + diffuse) * object_color;
  FragColor = vec4(result, 1.0);
}