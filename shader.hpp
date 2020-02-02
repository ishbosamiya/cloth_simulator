#ifndef SHADER_HPP
#define SHADER_HPP

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

using namespace std;

class Shader {
 public:
  unsigned int ID;

  Shader(const string &vertex_path, const string &fragment_path)
  {
    string vertex_code;
    string fragment_code;
    ifstream v_fin; /* vertex shader input stream */
    ifstream f_fin; /* fragment shader input stream */
    v_fin.exceptions(ifstream::failbit | ifstream::badbit);
    f_fin.exceptions(ifstream::failbit | ifstream::badbit);

    try {
      v_fin.open(vertex_path.c_str());
      f_fin.open(fragment_path.c_str());

      stringstream v_stream, f_stream;
      v_stream << v_fin.rdbuf();
      f_stream << f_fin.rdbuf();

      v_fin.close();
      f_fin.close();

      vertex_code = v_stream.str();
      fragment_code = f_stream.str();
    }
    catch (ifstream::failure e) {
      cout << "error: shader file couldn't be read!" << endl;
    }
    const char *v_code = vertex_code.c_str();
    const char *f_code = fragment_code.c_str();

    unsigned int vertex;
    vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &v_code, NULL);
    glCompileShader(vertex);
    checkCompileErrors(vertex, "VERTEX");

    unsigned int fragment;
    fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &f_code, NULL);
    glCompileShader(fragment);
    checkCompileErrors(fragment, "FRAGMENT");

    ID = glCreateProgram();
    glAttachShader(ID, vertex);
    glAttachShader(ID, fragment);
    glLinkProgram(ID);
    checkCompileErrors(ID, "PROGRAM");

    glDeleteShader(vertex);
    glDeleteShader(fragment);
  }

  void use()
  {
    glUseProgram(ID);
  }

  void setBool(const string &name, bool value) const
  {
    glUniform1i(glGetUniformLocation(ID, name.c_str()), (int)value);
  }

  void setInt(const string &name, int value) const
  {
    glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
  }

  void setFloat(const string &name, float value) const
  {
    glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
  }

  void setVec2(const std::string &name, const glm::vec2 &value) const
  {
    glUniform2fv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
  }
  void setVec2(const std::string &name, float x, float y) const
  {
    glUniform2f(glGetUniformLocation(ID, name.c_str()), x, y);
  }
  // ------------------------------------------------------------------------
  void setVec3(const std::string &name, const glm::vec3 &value) const
  {
    glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
  }
  void setVec3(const std::string &name, float x, float y, float z) const
  {
    glUniform3f(glGetUniformLocation(ID, name.c_str()), x, y, z);
  }
  // ------------------------------------------------------------------------
  void setVec4(const std::string &name, const glm::vec4 &value) const
  {
    glUniform4fv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
  }
  void setVec4(const std::string &name, float x, float y, float z, float w)
  {
    glUniform4f(glGetUniformLocation(ID, name.c_str()), x, y, z, w);
  }
  // ------------------------------------------------------------------------
  void setMat2(const std::string &name, const glm::mat2 &mat) const
  {
    glUniformMatrix2fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
  }
  // ------------------------------------------------------------------------
  void setMat3(const std::string &name, const glm::mat3 &mat) const
  {
    glUniformMatrix3fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
  }
  // ------------------------------------------------------------------------
  void setMat4(const std::string &name, const glm::mat4 &mat) const
  {
    glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
  }

 private:
  void checkCompileErrors(unsigned int shader, string type)
  {
    int success;
    char infoLog[1024];
    if (type == "PROGRAM") {
      glGetProgramiv(shader, GL_LINK_STATUS, &success);
      if (!success) {
        glGetProgramInfoLog(shader, 1024, NULL, infoLog);
        cout << "error: PROGRAM_LINKING_ERROR of type: " << type << endl << infoLog << endl;
      }
    }
    else {
      glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
      if (!success) {
        glGetShaderInfoLog(shader, 1024, NULL, infoLog);
        cout << "error: SHADER_COMPILATION_ERROR of type: " << type << endl << infoLog << endl;
      }
    }
  }
};

static Shader &defaultShader()
{
  static Shader default_shader("shaders/default.vert", "shaders/default.frag");
  return default_shader;
}

#endif
