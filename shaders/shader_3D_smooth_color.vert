#version 330 core

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

in vec3 pos;
in vec4 color;

out vec4 finalColor;

void main()
{
  gl_Position = projection * view * model * vec4(pos, 1.0);
  finalColor = color;
}
