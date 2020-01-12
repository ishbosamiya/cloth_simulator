#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "cloth_mesh.hpp"
#include "shader.hpp"
#include "camera.hpp"

using namespace std;

void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void mouse_callback(GLFWwindow *window, double xpos, double ypos);
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float last_x = SCR_WIDTH / 2.0f;
float last_y = SCR_HEIGHT / 2.0f;
bool first_mouse = true;

float delta_time = 0.0f;
float last_frame = 0.0f;

int main()
{
  // glfw: initialize and configure
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  // glfw window creation
  GLFWwindow *window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Cloth Simulator", NULL, NULL);
  if (window == NULL) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

  glfwSetCursorPosCallback(window, mouse_callback);
  glfwSetScrollCallback(window, scroll_callback);

  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

  // glad: load all OpenGL function pointers
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  glEnable(GL_DEPTH_TEST);

  // build and compile our shader program
  Shader shader("shader.vs", "shader.fs");

  // set up vertex data (and buffer(s)) and configure vertex attributes
  float vertices[] = {-0.5f, -0.5f, -0.5f, 0.0f, 0.0f,

                      0.5f,  -0.5f, -0.5f, 1.0f, 0.0f,

                      0.5f,  0.5f,  -0.5f, 1.0f, 1.0f,

                      0.5f,  0.5f,  -0.5f, 1.0f, 1.0f,

                      -0.5f, 0.5f,  -0.5f, 0.0f, 1.0f,

                      -0.5f, -0.5f, -0.5f, 0.0f, 0.0f,

                      -0.5f, -0.5f, 0.5f,  0.0f, 0.0f,

                      0.5f,  -0.5f, 0.5f,  1.0f, 0.0f,

                      0.5f,  0.5f,  0.5f,  1.0f, 1.0f,

                      0.5f,  0.5f,  0.5f,  1.0f, 1.0f,

                      -0.5f, 0.5f,  0.5f,  0.0f, 1.0f,

                      -0.5f, -0.5f, 0.5f,  0.0f, 0.0f,

                      -0.5f, 0.5f,  0.5f,  1.0f, 0.0f,

                      -0.5f, 0.5f,  -0.5f, 1.0f, 1.0f,

                      -0.5f, -0.5f, -0.5f, 0.0f, 1.0f,

                      -0.5f, -0.5f, -0.5f, 0.0f, 1.0f,

                      -0.5f, -0.5f, 0.5f,  0.0f, 0.0f,

                      -0.5f, 0.5f,  0.5f,  1.0f, 0.0f,

                      0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

                      0.5f,  0.5f,  -0.5f, 1.0f, 1.0f,

                      0.5f,  -0.5f, -0.5f, 0.0f, 1.0f,

                      0.5f,  -0.5f, -0.5f, 0.0f, 1.0f,

                      0.5f,  -0.5f, 0.5f,  0.0f, 0.0f,

                      0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

                      -0.5f, -0.5f, -0.5f, 0.0f, 1.0f,

                      0.5f,  -0.5f, -0.5f, 1.0f, 1.0f,

                      0.5f,  -0.5f, 0.5f,  1.0f, 0.0f,

                      0.5f,  -0.5f, 0.5f,  1.0f, 0.0f,

                      -0.5f, -0.5f, 0.5f,  0.0f, 0.0f,

                      -0.5f, -0.5f, -0.5f, 0.0f, 1.0f,

                      -0.5f, 0.5f,  -0.5f, 0.0f, 1.0f,

                      0.5f,  0.5f,  -0.5f, 1.0f, 1.0f,

                      0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

                      0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

                      -0.5f, 0.5f,  0.5f,  0.0f, 0.0f,

                      -0.5f, 0.5f,  -0.5f, 0.0f, 1.0f};
  // world space positions of our cubes
  glm::vec3 cube_positions[] = {glm::vec3(0.0f, 0.0f, 0.0f),
                                glm::vec3(2.0f, 5.0f, -15.0f),
                                glm::vec3(-1.5f, -2.2f, -2.5f),
                                glm::vec3(-3.8f, -2.0f, -12.3f),
                                glm::vec3(2.4f, -0.4f, -3.5f),
                                glm::vec3(-1.7f, 3.0f, -7.5f),
                                glm::vec3(1.3f, -2.0f, -2.5f),
                                glm::vec3(1.5f, 2.0f, -2.5f),
                                glm::vec3(1.5f, 0.2f, -1.5f),
                                glm::vec3(-1.3f, 1.0f, -1.5f)};

  unsigned int VBO, VAO;
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);
  // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure
  // vertex attributes(s).
  glBindVertexArray(VAO);

  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  // position
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  // uv
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  // render loop
  while (!glfwWindowShouldClose(window)) {
    // per-frame time logic
    float current_frame = glfwGetTime();
    delta_time = current_frame - last_frame;
    last_frame = current_frame;

    // input
    processInput(window);

    // render
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // use shader
    shader.use();

    // pass projection matrix to shader (note that in this case it could change every frame)
    glm::mat4 projection = glm::perspective(
        glm::radians(camera.zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
    shader.setMat4("projection", projection);

    // camera/view transformation
    glm::mat4 view = camera.getViewMatrix();
    shader.setMat4("view", view);

    glBindVertexArray(VAO);  // seeing as we only have a single VAO there's no need to bind it
                             // every time, but we'll do so to keep
                             // things a bit more organized
    for (int i = 0; i < 10; i++) {
      glm::mat4 model = glm::mat4(1.0f);
      model = glm::translate(model, cube_positions[i]);
      float angle = 20.0f * i;
      model = glm::rotate(model, glm::radians(angle), glm::vec3(1.0f, 0.3f, 0.5f));
      shader.setMat4("model", model);
      glDrawArrays(GL_TRIANGLES, 0, 36);
    }

    // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // optional: de-allocate all resources once they've outlived their purpose:
  glDeleteVertexArrays(1, &VAO);
  glDeleteBuffers(1, &VBO);

  // glfw: terminate, clearing all previously allocated GLFW resources.
  glfwTerminate();
  return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react
// accordingly
void processInput(GLFWwindow *window)
{
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, true);
  }

  if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
    camera.processKeyboard(FORWARD, delta_time);
  }

  if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
    camera.processKeyboard(BACKWARD, delta_time);
  }

  if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
    camera.processKeyboard(LEFT, delta_time);
  }

  if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
    camera.processKeyboard(RIGHT, delta_time);
  }
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
void framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
  // make sure the viewport matches the new window dimensions; note that width and
  // height will be significantly larger than specified on retina displays.
  glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
void mouse_callback(GLFWwindow *window, double xpos, double ypos)
{
  if (first_mouse) {
    last_x = xpos;
    last_y = ypos;
    first_mouse = false;
  }

  float xoffset = xpos - last_x;
  float yoffset = last_y - ypos;  // reversed since y-coordinates go from bottom to top

  last_x = xpos;
  last_y = ypos;

  camera.processMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
  camera.processMouseScroll(yoffset);
}
