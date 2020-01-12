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

  Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),
         glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
         float yaw = YAW,
         float pitch = PITCH)
      : front(glm::vec3(0.0f, 0.0f, -1.0f)),
        movement_speed(SPEED),
        mouse_sensitivity(SENSITIVITY),
        zoom(ZOOM)
  {
    this->position = position;
    this->world_up = up;
    this->yaw = yaw;
    this->pitch = pitch;

    updateCameraVectors();
  }

  glm::mat4 getViewMatrix()
  {
    return glm::lookAt(position, position + front, up);
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

 private:
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
