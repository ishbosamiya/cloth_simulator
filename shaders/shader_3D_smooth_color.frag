#version 330 core

in vec4 finalColor;
out vec4 fragColor;

void main()
{
  fragColor = finalColor;
}
