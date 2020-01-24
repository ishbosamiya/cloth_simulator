#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
  gl_Position = projection * view * model * vec4(aPos, 1.0);
  gl_Position = vec4(gl_Position.xy, gl_Position.z - 0.005, gl_Position.w);
}
