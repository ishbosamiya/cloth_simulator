#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <vector>

enum CameraMovement { FORWARD, BACKWARD, LEFT, RIGHT };

const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 2.5f;
const float SENSITIVITY = 0.1f;
const float ZOOM = 45.0f;

class Camera {
 public:
  glm::vec3 position;
  glm::vec3 front;
  glm::vec3 up;
  glm::vec3 right;
  glm::vec3 world_up;

  float yaw;
  float pitch;

  float movement_speed;
  float mouse_sensitivity;
  float zoom;

  unsigned int width;
  unsigned int height;

  Camera(unsigned int width,
         unsigned int height,
         glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),
         glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
         float yaw = YAW,
         float pitch = PITCH)
      : front(glm::vec3(0.0f, 0.0f, -1.0f)),
        movement_speed(SPEED),
        mouse_sensitivity(SENSITIVITY),
        zoom(ZOOM)
  {
    this->width = width;
    this->height = height;
    this->position = position;
    this->world_up = up;
    this->yaw = yaw;
    this->pitch = pitch;

    updateCameraVectors();

    this->initial_position = this->position;
    this->initial_front = this->front;
    this->initial_up = this->up;
    this->initial_right = this->right;
    this->initial_world_up = this->world_up;

    this->initial_yaw = this->yaw;
    this->initial_pitch = this->pitch;

    this->initial_movement_speed = this->movement_speed;
    this->initial_mouse_sensitivity = this->mouse_sensitivity;
    this->initial_zoom = this->zoom;
  }

  void reset()
  {
    this->position = this->initial_position;
    this->front = this->initial_front;
    this->up = this->initial_up;
    this->right = this->initial_right;
    this->world_up = this->initial_world_up;

    this->yaw = this->initial_yaw;
    this->pitch = this->initial_pitch;

    this->movement_speed = this->initial_movement_speed;
    this->mouse_sensitivity = this->initial_mouse_sensitivity;
    this->zoom = this->initial_zoom;

    updateCameraVectors();
  }

  inline glm::mat4 getViewMatrix()
  {
    return glm::lookAt(position, position + front, up);
  }

  inline glm::mat4 getProjectionMatrix()
  {
    return glm::perspective(glm::radians(zoom), (float)width / (float)height, 0.1f, 100.0f);
  }

  /* refer to
   * https://gamedev.stackexchange.com/questions/81481/how-to-implement-camera-pan-like-in-maya
   * for more details */
  void pan(float mouse_start_x,
           float mouse_start_y,
           float mouse_end_x,
           float mouse_end_y,
           float len = 1.0f)
  {
    float clipX = mouse_start_x * 2.0 / width - 1.0;
    float clipY = 1.0 - mouse_start_y * 2.0 / height;

    float clipEndX = mouse_end_x * 2.0 / width - 1.0;
    float clipEndY = 1.0 - mouse_end_y * 2.0 / height;

    // convert begin and end mouse positions into world space
    glm::mat4 inverseMVP = glm::inverse(getProjectionMatrix() * getViewMatrix());
    glm::vec4 outVector = inverseMVP * glm::vec4(clipX, clipY, 0.0, 1.0);
    glm::vec3 worldPos(
        outVector.x / outVector.w, outVector.y / outVector.w, outVector.z / outVector.w);

    glm::vec4 outEndVec = inverseMVP * glm::vec4(clipEndX, clipEndY, 0.0, 1.0);
    glm::vec3 worldPos2(
        outEndVec.x / outEndVec.w, outEndVec.y / outEndVec.w, outEndVec.z / outEndVec.w);

    glm::vec3 dir = worldPos2 - worldPos;

    glm::vec3 offset = glm::length(dir) * glm::normalize(dir) * zoom * len / 2.0f;
    position -= offset;
  }

  void moveForward(float mouse_start_y, float mouse_end_y)
  {
    float clipY = 1.0 - mouse_start_y * 2.0 / height;
    float clipEndY = 1.0 - mouse_end_y * 2.0 / height;

    float move_by = clipEndY - clipY;

    position += front * move_by;
  }

  void processKeyboard(CameraMovement direction, float delta_time)
  {
    float velocity = movement_speed * delta_time;
    if (direction == FORWARD) {
      position += front * velocity;
    }
    if (direction == BACKWARD) {
      position -= front * velocity;
    }
    if (direction == LEFT) {
      position -= right * velocity;
    }
    if (direction == RIGHT) {
      position += right * velocity;
    }
  }

  void processMouseMovement(float x_offset, float y_offset, GLboolean constrain_pitch = true)
  {
    x_offset *= mouse_sensitivity;
    y_offset *= mouse_sensitivity;

    yaw += x_offset;
    pitch += y_offset;

    if (constrain_pitch) {
      if (pitch > 89.0f) {
        pitch = 89.0f;
      }

      if (pitch < -89.0f) {
        pitch = -89.0f;
      }
    }

    updateCameraVectors();
  }

  void processMouseScroll(float y_offset)
  {
    if (zoom >= 1.0f && zoom <= 45.0f) {
      zoom -= y_offset;
    }
    if (zoom <= 1.0f) {
      zoom = 1.0f;
    }
    if (zoom >= 45.0f) {
      zoom = 45.0f;
    }
  }

  glm::vec3 getRaycastDirection(float mouse_x, float mouse_y)
  {
    float x = (2.0f * mouse_x) / width - 1.0f;
    float y = 1.0f - (2.0f * mouse_y) / height;
    float z = 1.0f;
    glm::vec3 ray_nds = glm::vec3(x, y, z);
    glm::vec4 ray_clip = glm::vec4(ray_nds.x, ray_nds.y, -1.0, 1.0);

    glm::vec4 ray_eye = glm::inverse(getProjectionMatrix()) * ray_clip;
    ray_eye = glm::vec4(ray_eye.x, ray_eye.y, -1.0, 0.0);

    glm::vec4 ray_wor = (glm::inverse(getViewMatrix()) * ray_eye);
    glm::vec3 result = glm::vec3(ray_wor.x, ray_wor.y, ray_wor.z);
    result = glm::normalize(result);
    return result;
  }

 private:
  glm::vec3 initial_position;
  glm::vec3 initial_front;
  glm::vec3 initial_up;
  glm::vec3 initial_right;
  glm::vec3 initial_world_up;

  float initial_yaw;
  float initial_pitch;

  float initial_movement_speed;
  float initial_mouse_sensitivity;
  float initial_zoom;

  void updateCameraVectors()
  {
    glm::vec3 front;
    front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    front.y = sin(glm::radians(pitch));
    front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    this->front = glm::normalize(front);

    right = glm::normalize(glm::cross(front, world_up));
    up = glm::normalize(glm::cross(right, front));
  }
};

#endif
