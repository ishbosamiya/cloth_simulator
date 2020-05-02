#include <iostream>
#include <string>
#include <cstdio>
#include <cstdlib>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "cloth_mesh.hpp"
#include "shader.hpp"
#include "camera.hpp"
#include "opengl_mesh.hpp"
#include "simulation.hpp"
#include "text.hpp"
#include "gpu_immediate.hpp"

using namespace std;

void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void mouse_callback(GLFWwindow *window, double xpos, double ypos);
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);
void handlePinConstraints(GLFWwindow *window, Simulation *simulation, Camera *camera);

void immTest(glm::mat4 &projection, glm::mat4 view)
{
  static Shader smooth_shader("shaders/shader_3D_smooth_color.vert",
                              "shaders/shader_3D_smooth_color.frag");
  glm::mat4 model = glm::mat4(1.0);
  smooth_shader.use();
  smooth_shader.setMat4("projection", projection);
  smooth_shader.setMat4("view", view);
  smooth_shader.setMat4("model", model);

  GPUVertFormat *format = immVertexFormat();
  uint pos = format->addAttribute("pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
  uint col = format->addAttribute("color", GPU_COMP_F32, 4, GPU_FETCH_FLOAT);

  immBegin(GPU_PRIM_TRIS, 3, &smooth_shader);

  immAttr4f(col, 1.0, 0.0, 0.0, 1.0);
  immVertex3f(pos, 0, 0, -1);

  immAttr4f(col, 0.0, 1.0, 0.0, 1.0);
  immVertex3f(pos, 1, 1, -1);

  immAttr4f(col, 0.0, 0.0, 1.0, 1.0);
  immVertex3f(pos, 1.5, 0, -1);

  immEnd();
}

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

Camera camera(SCR_WIDTH, SCR_HEIGHT, glm::vec3(0.0f, 0.0f, 3.0f));
float last_x = SCR_WIDTH / 2.0f;
float last_y = SCR_HEIGHT / 2.0f;
float last_cursor_x = last_x;
float last_cursor_y = last_y;
bool first_mouse = true;

float delta_time = 0.0f;
float last_frame = 0.0f;

bool simulation_pause = true;

bool draw_constraints_stretch = false;
bool draw_constraints_bending = false;

int main()
{
  srand((int)glfwGetTime());
  // glfw: initialize and configure
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  /* glfwWindowHint(GLFW_DOUBLEBUFFER, GL_FALSE); */
  /* glfwSwapInterval(0); */

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

  GLFWcursor *cursor = glfwCreateStandardCursor(GLFW_CROSSHAIR_CURSOR);
  glfwSetCursor(window, cursor);

  // glad: load all OpenGL function pointers
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  glEnable(GL_DEPTH_TEST);
  /* This is mainly for the text, might be causing a slow down for the
   * rest of it */
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  /* Initialize gpu_immediate work-alike */
  immInit();
  immActivate();

  glm::vec3 light_dir(-0.7f, -1.0f, -0.7f);
  // build and compile our shader program
  Shader directional_light_shader("shaders/directional_light.vert",
                                  "shaders/directional_light.frag");
  Shader light_shader("shaders/light.vert", "shaders/light.frag");
  ClothMesh mesh("something.obj", Vec3(0, 0, 0), Vec3(1.0), &directional_light_shader);
  mesh.setCoeffFriction(0.3);
  Mesh light("light.obj", glmVec3ToVec3(light_dir * glm::vec3(-2.2f)), Vec3(0.2), &light_shader);
  Simulation simulation(&mesh);
  /* Sphere ob_mesh(0.3, Vec3(0, -0.5, 0), &directional_light_shader); */
  Mesh ob_mesh("obstacle.obj", Vec3(0, -1.2, 0), Vec3(1.0), &directional_light_shader);
  simulation.addObstacleMesh(&ob_mesh);

  /* Text initialization */
  Text text;
  text.loadFont("/usr/share/fonts/truetype/ubuntu/Ubuntu-M.ttf", "ubuntu", 0, 48);
  Shader text_shader("shaders/text_shader.vert", "shaders/text_shader.frag");
  text_shader.use();
  text_shader.setVec3("textColor", 1.0, 1.0, 1.0);
  text_shader.setMat4(
      "projection",
      glm::ortho(0.0f, static_cast<GLfloat>(SCR_WIDTH), 0.0f, static_cast<GLfloat>(SCR_HEIGHT)));

  // render loop
  unsigned int frame_count = 0;
  float initial_time = glfwGetTime();
  while (!glfwWindowShouldClose(window)) {
    frame_count++;

    // per-frame time logic
    float current_frame = glfwGetTime();
    delta_time = current_frame - last_frame;
    last_frame = current_frame;

    float fps = 1.0f / delta_time;
    float avg_fps = frame_count / (current_frame - initial_time);
    if (frame_count > 240) {
      frame_count = 0;
      initial_time = glfwGetTime();
    }

    // input
    processInput(window);
    handlePinConstraints(window, &simulation, &camera);

    // render
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // use shader
    /* TODO(ish): update all shaders, design a better system to do
       this */
    defaultShader().use();
    defaultShader().setMat4("projection", camera.getProjectionMatrix());
    defaultShader().setMat4("view", camera.getViewMatrix());
    directional_light_shader.use();
    directional_light_shader.setVec3("viewPos", camera.position);
    directional_light_shader.setVec3("material.color", 0.3f, 0.2f, 0.7f);
    glm::vec3 specular(0.3f);
    directional_light_shader.setVec3("material.specular", specular);
    directional_light_shader.setFloat("material.shininess", 4.0f);
    directional_light_shader.setVec3("light.direction", light_dir);
    directional_light_shader.setVec3("light.ambient", 0.3f, 0.3f, 0.3f);
    directional_light_shader.setVec3("light.diffuse", 1.0f, 1.0f, 1.0f);
    directional_light_shader.setVec3("light.specular", 1.0f, 1.0f, 1.0f);

    glm::mat4 projection = camera.getProjectionMatrix();
    glm::mat4 view = camera.getViewMatrix();

    directional_light_shader.setMat4("projection", projection);
    directional_light_shader.setMat4("view", view);

    mesh.draw();

    light_shader.use();
    light_shader.setMat4("projection", projection);
    light_shader.setMat4("view", view);

    light.draw();

    directional_light_shader.use();
    directional_light_shader.setVec3("material.color", 0.7f, 0.5f, 0.5f);
    ob_mesh.draw();

    if (!simulation_pause) {
      simulation.update();
      /* char output_filename[512]; */
      /* sprintf(output_filename, "/tmp/objs/obj_%03d.obj", frame_count); */
      /* mesh.saveObj(output_filename); */
    }

    simulation.drawConstraints(
        projection, view, draw_constraints_stretch, draw_constraints_bending);

    char fps_text[25];
    snprintf(fps_text, 25, "fps: %.2f", fps);
    text.renderText(text_shader, fps_text, "ubuntu", 7.0, SCR_HEIGHT - 20, 0.3);
    snprintf(fps_text, 25, "avg_fps: %.2f", avg_fps);
    text.renderText(text_shader, fps_text, "ubuntu", 7.0, SCR_HEIGHT - 40, 0.3);

    immTest(projection, view);

    // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  /* terminate gpu_immediate work-alike */
  immDeactivate();
  immDestroy();

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

  if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
    if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS) {
      camera.reset();
    }
  }

  if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS) {
    draw_constraints_stretch = !draw_constraints_stretch;
  }
  if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS) {
    draw_constraints_bending = !draw_constraints_bending;
  }

  if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS) {
    simulation_pause = false;
  }
  else {
    simulation_pause = true;
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
  float yoffset = last_y - ypos;  // reversed since y-coordinates go
                                  // from bottom to top

  int mouse_state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE);
  if (mouse_state == GLFW_PRESS) {
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
      camera.pan(last_x, last_y, xpos, ypos, 1.0f);
    }
    else if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
      camera.moveForward(last_y, ypos);
    }
    else {
      camera.processMouseMovement(xoffset, yoffset);
    }
  }

  last_x = xpos;
  last_y = ypos;
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
  camera.processMouseScroll(yoffset);
}

void handlePinConstraints(GLFWwindow *window, Simulation *simulation, Camera *camera)
{
  static bool still_pressed = false;
  int mouse_state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
  double x, y;
  glfwGetCursorPos(window, &x, &y);
  if (mouse_state == GLFW_PRESS) {
    if (!still_pressed) {
      still_pressed = true;
    }
  }
  if (mouse_state == GLFW_RELEASE) {
    if (still_pressed) {
      simulation->tryToTogglePinConstraint(Vec3(camera->position),
                                           Vec3(camera->getRaycastDirection(x, y)));

      still_pressed = false;
    }
  }
}
