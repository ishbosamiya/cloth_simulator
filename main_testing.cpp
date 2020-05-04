#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "camera.hpp"
#include "shader.hpp"
#include "gpu_immediate.hpp"
#include "text.hpp"

/* settings */
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

Camera camera(SCR_WIDTH, SCR_HEIGHT, glm::vec3(0.0f, 0.0f, 3.0f));

void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void mouse_callback(GLFWwindow *window, double xpos, double ypos);
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);

bool first_mouse = true;
float delta_time = 0.0f;
float last_frame = 0.0f;
float last_x = SCR_WIDTH / 2.0f;
float last_y = SCR_HEIGHT / 2.0f;

int main()
{
  /* glfw: initialize and configure */
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  /* glfw window creation */
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

  /* glad: load all OpenGL function pointers */
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  glEnable(GL_DEPTH_TEST);
  /* This is mainly for the text, might be causing a slow down for the
   * rest of it, but shouldn't matter all that much considering this
   * is not a game engine */
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  /* Initialize gpu_immediate work-alike */
  immInit();
  immActivate();

  /* Text initialization */
  Text text;
  text.loadFont("/usr/share/fonts/truetype/ubuntu/Ubuntu-M.ttf", "ubuntu", 0, 48);
  Shader text_shader("shaders/text_shader.vert", "shaders/text_shader.frag");
  text_shader.use();
  text_shader.setVec3("textColor", 1.0, 1.0, 1.0);
  text_shader.setMat4(
      "projection",
      glm::ortho(0.0f, static_cast<GLfloat>(SCR_WIDTH), 0.0f, static_cast<GLfloat>(SCR_HEIGHT)));

  /* render loop */
  unsigned int frame_count = 0;
  float initial_time = glfwGetTime();
  while (!glfwWindowShouldClose(window)) {
    frame_count++;

    /* per-frame time logic */
    float current_frame = glfwGetTime();
    delta_time = current_frame - last_frame;
    last_frame = current_frame;

    float fps = 1.0f / delta_time;
    float avg_fps = frame_count / (current_frame - initial_time);
    if (frame_count > 240) {
      frame_count = 0;
      initial_time = glfwGetTime();
    }

    /* input */
    processInput(window);

    /* render */
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glm::mat4 projection = camera.getProjectionMatrix();
    glm::mat4 view = camera.getViewMatrix();

    char fps_text[25];
    snprintf(fps_text, 25, "fps: %.2f", fps);
    text.renderText(text_shader, fps_text, "ubuntu", 7.0, SCR_HEIGHT - 20, 0.3);
    snprintf(fps_text, 25, "avg_fps: %.2f", avg_fps);
    text.renderText(text_shader, fps_text, "ubuntu", 7.0, SCR_HEIGHT - 40, 0.3);

    /* glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.) */
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  /* terminate gpu_immediate work-alike */
  immDeactivate();
  immDestroy();

  /* glfw: terminate, clearing all previously allocated GLFW resources. */
  glfwTerminate();
  return 0;
}

/* process all input: query GLFW whether relevant keys are pressed/released this frame and react
 * accordingly */
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
}

/* glfw: whenever the window size changed (by OS or user resize) this callback function executes
 */
void framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
  /* make sure the viewport matches the new window dimensions; note that width and height will be
   * significantly larger than specified on retina displays. */
  glViewport(0, 0, width, height);
}

/* glfw: whenever the mouse moves, this callback is called */
void mouse_callback(GLFWwindow *window, double xpos, double ypos)
{
  if (first_mouse) {
    last_x = xpos;
    last_y = ypos;
    first_mouse = false;
  }

  float xoffset = xpos - last_x;
  float yoffset = last_y - ypos; /* reversed since y-coordinates go from bottom to top */

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

/* glfw: whenever the mouse scroll wheel scrolls, this callback is called */
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
  camera.processMouseScroll(yoffset);
}
