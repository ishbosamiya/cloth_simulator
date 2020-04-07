#ifndef TEXT_HPP
#define TEXT_HPP

#include <iostream>
#include <string>
#include <map>
#include <ft2build.h>
#include FT_FREETYPE_H
#include <glad/glad.h>

#include "math.hpp"
#include "shader.hpp"

using namespace std;

struct Character {
  GLuint textureID;   /* ID handle of the glyph texture */
  glm::ivec2 size;    /* Size of glyph */
  glm::ivec2 bearing; /* Offset from baseline to left/top of glyph */
  GLuint advance;     /* Horizontal offset to advance to next glyph */
};

typedef map<GLchar, Character> Characters;

class Text {
 public:
  FT_Library ft_lib;
  map<string, Characters> fonts;

  Text()
  {
    if (FT_Init_FreeType(&ft_lib)) { /* Some weird way where the
                                        return is true when the function fails */
      cout << "error: freetype: library not initialized!" << endl;
      return;
    }
  }

  /* font -> location of font
   * width -> width of characters, 0 for dynamic
   * height -> height of characters */
  void loadFont(string font, string key, int width, int height, GLubyte num_char = 128)
  {
    /* If font is already loaded don't reload */
    if (fonts.find(key) != fonts.end()) {
      return;
    }

    FT_Face face;
    if (FT_New_Face(ft_lib, font.c_str(), 0, &face)) {
      cout << "error: freetype: font " << key << " not initialized!" << endl;
      return;
    }

    /* Disable byte alignment restriction */
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    FT_Set_Pixel_Sizes(face, width, height);

    Characters characters;

    for (GLubyte i = 0; i < num_char; i++) {
      if (FT_Load_Char(face, i, FT_LOAD_RENDER)) {
        cout << "error: freetype: Glyph not loaded for " << i << "!" << endl;
        continue;
      }
      GLuint texture;
      glGenTextures(1, &texture);
      glBindTexture(GL_TEXTURE_2D, texture);
      glTexImage2D(GL_TEXTURE_2D,
                   0,
                   GL_RED,
                   face->glyph->bitmap.width,
                   face->glyph->bitmap.rows,
                   0,
                   GL_RED,
                   GL_UNSIGNED_BYTE,
                   face->glyph->bitmap.buffer);

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      Character character = {texture,
                             glm::ivec2(face->glyph->bitmap.width, face->glyph->bitmap.rows),
                             glm::ivec2(face->glyph->bitmap_left, face->glyph->bitmap_top),
                             (GLuint)face->glyph->advance.x};
      characters.insert(std::pair<GLchar, Character>(i, character));
    }
    glBindTexture(GL_TEXTURE_2D, 0);

    /* Store the characters mapping textures in the vector for later use */
    fonts[key] = characters;

    FT_Done_Face(face);
  }

  void unloadFont(string key)
  {
    if (fonts.find(key) != fonts.end()) {
      /* TODO(ish): need to unbind the textures */
    }
    else {
      cout << "warning: freetype: trying to unload font that is not loaded!" << endl;
    }
  }

  void renderText(Shader &shader, string text, string key, GLfloat x, GLfloat y, GLfloat scale)
  {
    if (fonts.find(key) == fonts.end()) {
      cout << "error: font was not loaded!" << endl;
      return;
    }
    /* TODO(ish): this can be optimized heavily, currently
       implementing it as a hack instead of using the GLMesh */
    unsigned int VAO;
    unsigned int VBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 6 * 4, NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    shader.use();
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(VAO);

    Characters characters = fonts[key];

    // Iterate through all characters
    std::string::const_iterator c;
    for (c = text.begin(); c != text.end(); c++) {
      Character ch = characters[*c];

      GLfloat xpos = x + ch.bearing.x * scale;
      GLfloat ypos = y - (ch.size.y - ch.bearing.y) * scale;

      GLfloat w = ch.size.x * scale;
      GLfloat h = ch.size.y * scale;
      // Update VBO for each character
      GLfloat vertices[6][4] = {{xpos, ypos + h, 0.0, 0.0},
                                {xpos, ypos, 0.0, 1.0},
                                {xpos + w, ypos, 1.0, 1.0},

                                {xpos, ypos + h, 0.0, 0.0},
                                {xpos + w, ypos, 1.0, 1.0},
                                {xpos + w, ypos + h, 1.0, 0.0}};
      /* Render glyph texture over quad */
      glBindTexture(GL_TEXTURE_2D, ch.textureID);
      /* Update content of VBO memory */
      glBindBuffer(GL_ARRAY_BUFFER, VBO);
      glBufferSubData(
          GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices); /* Be sure to use glBufferSubData
                                                            * and not glBufferData */

      glBindBuffer(GL_ARRAY_BUFFER, 0);
      /* Render quad */
      glDrawArrays(GL_TRIANGLES, 0, 6);
      /* Now advance cursors for next glyph (note that advance is
       * number of 1/64 pixels) */
      x += (ch.advance >> 6) * scale; /* Bitshift by 6 to get value in pixels (2^6 = 64 (divide
                                       * amount of 1/64th pixels by
                                       * 64 to get amount of pixels)) */
    }
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  ~Text()
  {
    FT_Done_FreeType(ft_lib);
  }
};

#endif
