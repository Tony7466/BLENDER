#version 330 core

// Vertex Shader
#shader vertex
layout(location = 0) in vec3 aPos;

void main()
{
    gl_Position = vec4(aPos, 1.0);
}

// Fragment Shader
#shader fragment
out vec4 FragColor;

void main()
{
    FragColor = vec4(1.0, 0.5, 0.2, 1.0); // Couleur orange
}
