#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "cloth_mesh.hpp"
#include "shader.hpp"
#include "camera.hpp"
#include "opengl_mesh.hpp"

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
  Shader directional_light_shader("shaders/directional_light.vert",
                                  "shaders/directional_light.frag");
  Shader light_shader("shaders/light.vert", "shaders/light.frag");
  ClothMesh mesh;
  mesh.loadObj("something.obj");
  ClothMesh light;
  light.loadObj("light.obj");

  glm::vec3 light_pos(1.0f, 1.0f, 1.0f);

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
    directional_light_shader.use();
    directional_light_shader.setVec3("viewPos", camera.position);
    directional_light_shader.setVec3("material.color", 0.3f, 0.2f, 0.7f);
    glm::vec3 specular(0.3f);
    directional_light_shader.setVec3("material.specular", specular);
    directional_light_shader.setFloat("material.shininess", 4.0f);
    glm::vec3 light_dir(-0.7f, -1.0f, -0.7f);
    directional_light_shader.setVec3("light.direction", light_dir);
    directional_light_shader.setVec3("light.ambient", 0.3f, 0.3f, 0.3f);
    directional_light_shader.setVec3("light.diffuse", 1.0f, 1.0f, 1.0f);
    directional_light_shader.setVec3("light.specular", 1.0f, 1.0f, 1.0f);

    glm::mat4 projection = glm::perspective(
        glm::radians(camera.zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
    glm::mat4 view = camera.getViewMatrix();

    directional_light_shader.setMat4("projection", projection);
    directional_light_shader.setMat4("view", view);

    glm::mat4 model = glm::mat4(1.0f);
    directional_light_shader.setMat4("model", model);

    mesh.draw();

    light_shader.use();
    model = glm::mat4(1.0f);
    model = glm::translate(model, light_dir * glm::vec3(-2.2f));
    model = glm::scale(model, glm::vec3(0.2f));
    light_shader.setMat4("projection", projection);
    light_shader.setMat4("view", view);
    light_shader.setMat4("model", model);

    light.draw();

    // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

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
