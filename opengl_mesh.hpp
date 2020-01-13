#ifndef OPENGL_MESH
#define OPENGL_MESH

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <vector>

using namespace std;

struct GLVertex {
  glm::vec3 x;  /* position */
  glm::vec3 n;  /* normal */
  glm::vec2 uv; /* texture coordinate */
};

class GLMesh {
 public:
  vector<GLVertex> verts;
  vector<unsigned int> indices;
  unsigned int VAO;

  GLMesh(vector<GLVertex> verts, vector<unsigned int> indices)
  {
    this->verts = verts;
    this->indices = indices;

    setupMesh();
  }

  void draw()
  {
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
  }

 private:
  unsigned int VBO, EBO;

  void setupMesh()
  {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(GLVertex), &verts[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 indices.size() * sizeof(unsigned int),
                 &indices[0],
                 GL_STATIC_DRAW);

    // positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLVertex), (void *)0);
    // normals
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(
        1, 3, GL_FLOAT, GL_FALSE, sizeof(GLVertex), (void *)offsetof(GLVertex, n));
    // uv
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(
        2, 2, GL_FLOAT, GL_FALSE, sizeof(GLVertex), (void *)offsetof(GLVertex, uv));

    glBindVertexArray(0);
  }
};

#endif
