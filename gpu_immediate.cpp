#include "gpu_immediate.hpp"

class GPUImmediate {
 public:
  /* current draw call */
  GLubyte *buffer_data;
  uint buffer_offset;
  uint buffer_bytes_mapped;
  uint vertex_len;
  bool strict_vertex_len;
  GPUPrimType prim_type;

  GPUVertFormat vertex_format;

  /* current vertex */
  uint vertex_idx;
  GLubyte *vertex_data;
  uint16_t unassigned_attr_bits; /* which attributes of current vertex have
                                  * not been given values? */

  GLuint vbo_id;
  GLuint vao_id;

  GPUAttrBinding attr_binding;
  uint16_t prev_enabled_attr_bits; /* <-- only affects this VAO, so we're ok */
};

/* size of internal buffer */
#define DEFAULT_INTERNAL_BUFFER_SIZE (4 * 1024 * 1024)
static uint imm_buffer_size = DEFAULT_INTERNAL_BUFFER_SIZE;
static bool initialized = false;
static GPUImmediate imm;

uchar GPUVertFormat::copyAttributeName(const char *name)
{
  /* strncpy does 110% of what we need; let's do exactly 100% */
  uchar name_offset = name_offset;
  char *name_copy = names + name_offset;
  uint available = GPU_VERT_ATTR_NAMES_BUF_LEN - name_offset;

  for (uint i = 0; i < available; i++) {
    const char c = name[i];
    name_copy[i] = c;
    if (c == '\0') {
      name_offset += (i + 1);
      break;
    }
  }
  return name_offset;
}

static GLenum convert_comp_type_to_gl(GPUVertCompType type)
{
  static const GLenum table[] = {
      [GPU_COMP_I8] = GL_BYTE,
      [GPU_COMP_U8] = GL_UNSIGNED_BYTE,
      [GPU_COMP_I16] = GL_SHORT,
      [GPU_COMP_U16] = GL_UNSIGNED_SHORT,
      [GPU_COMP_I32] = GL_INT,
      [GPU_COMP_U32] = GL_UNSIGNED_INT,

      [GPU_COMP_F32] = GL_FLOAT,

      [GPU_COMP_I10] = GL_INT_2_10_10_10_REV,
  };
  return table[type];
}

uint GPUVertFormat::addAttribute(const char *name,
                                 GPUVertCompType comp_type,
                                 uint comp_len,
                                 GPUVertFetchMode fetch_mode)
{
  name_len++; /* multiname support */

  const uint attr_id = attr_len++;
  GPUVertAttr *attr = &attrs[attr_id];

  attr->names[attr->name_len++] = copyAttributeName(name);
  attr->comp_type = comp_type;
  attr->gl_comp_type = convert_comp_type_to_gl(comp_type);
  attr->comp_len = (comp_type == GPU_COMP_I10) ?
                       4 :
                       comp_len; /* system needs 10_10_10_2 to be 4 or BGRA */
  attr->sz = attr->attrSZ();
  attr->offset = 0; /* offsets & stride are calculated later (during pack) */
  attr->fetch_mode = fetch_mode;

  return attr_id;
}

void GPUVertFormat::clear()
{
  attr_len = 0;
  packed = false;
  name_offset = 0;
  name_len = 0;

  for (uint i = 0; i < GPU_VERT_ATTR_MAX_LEN; i++) {
    attrs[i].name_len = 0;
  }
}

const char *GPUVertFormat::getAttributeName(const GPUVertAttr *attr, uint n_idx)
{
  return names + attr->names[n_idx];
}

int GPUVertFormat::getAttributeID(const char *name)
{
  for (int i = 0; i < attr_len; i++) {
    const GPUVertAttr *attr = &attrs[i];
    for (int j = 0; j < attr->name_len; j++) {
      const char *attr_name = getAttributeName(attr, j);
      if (strcmp(name, attr_name) == 0) {
        return i;
      }
    }
  }
  return -1;
}

static GLuint GPU_buf_alloc()
{
  GLuint new_buffer_id = 0;
  glGenBuffers(1, &new_buffer_id);
  return new_buffer_id;
}

static void GPU_buf_free(GLuint buf_id)
{
  glDeleteBuffers(1, &buf_id);
}

void immInit()
{
  memset(&imm, 0, sizeof(GPUImmediate));

  imm.vbo_id = GPU_buf_alloc();
  glBindBuffer(GL_ARRAY_BUFFER, imm.vbo_id);
  glBufferData(GL_ARRAY_BUFFER, imm_buffer_size, NULL, GL_DYNAMIC_DRAW);

  imm.prim_type = GPU_PRIM_NONE;
  imm.strict_vertex_len = true;

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  initialized = true;
}

void immDestroy()
{
  GPU_buf_free(imm.vbo_id);
  initialized = false;
}
