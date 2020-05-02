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

static inline GLenum convert_prim_type_to_gl(GPUPrimType prim_type)
{
  static GLenum table[10];
  table[GPU_PRIM_POINTS] = GL_POINTS;
  table[GPU_PRIM_LINES] = GL_LINES;
  table[GPU_PRIM_LINE_STRIP] = GL_LINE_STRIP;
  table[GPU_PRIM_LINE_LOOP] = GL_LINE_LOOP;
  table[GPU_PRIM_TRIS] = GL_TRIANGLES;
  table[GPU_PRIM_TRI_STRIP] = GL_TRIANGLE_STRIP;
  table[GPU_PRIM_TRI_FAN] = GL_TRIANGLE_FAN;
  table[GPU_PRIM_LINES_ADJ] = GL_LINES_ADJACENCY;
  table[GPU_PRIM_LINE_STRIP_ADJ] = GL_LINE_STRIP_ADJACENCY;
  table[GPU_PRIM_TRIS_ADJ] = GL_TRIANGLES_ADJACENCY;

  return table[prim_type];
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

uint GPUVertFormat::vertexBufferSize(uint vertex_len)
{
  return stride * vertex_len;
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

static uint padding(uint offset, uint alignment)
{
  const uint mod = offset % alignment;
  return (mod == 0) ? 0 : (alignment - mod);
}

void immBegin(GPUPrimType prim_type, uint vertex_len)
{
  imm.prim_type = prim_type;
  imm.vertex_len = vertex_len;
  imm.vertex_idx = 0;
  imm.unassigned_attr_bits = imm.attr_binding.enabled_bits;

  /* how many bytes do we need for this draw call? */
  const uint bytes_needed = imm.vertex_format.vertexBufferSize(vertex_len);

  glBindBuffer(GL_ARRAY_BUFFER, imm.vbo_id);

  /* does the current buffer have enough room? */
  const uint available_bytes = imm_buffer_size - imm.buffer_offset;

  bool recreate_buffer = false;
  if (bytes_needed > imm_buffer_size) {
    /* expand the internal buffer */
    imm_buffer_size = bytes_needed;
    recreate_buffer = true;
  }
  else if (bytes_needed < DEFAULT_INTERNAL_BUFFER_SIZE &&
           imm_buffer_size > DEFAULT_INTERNAL_BUFFER_SIZE) {
    /* shrink the internal buffer */
    imm_buffer_size = DEFAULT_INTERNAL_BUFFER_SIZE;
    recreate_buffer = true;
  }

  /* ensure vertex data is aligned */
  /* Might waste a little space, but it's safe. */
  const uint pre_padding = padding(imm.buffer_offset, imm.vertex_format.getStride());

  if (!recreate_buffer && ((bytes_needed + pre_padding) <= available_bytes)) {
    imm.buffer_offset += pre_padding;
  }
  else {
    /* orphan this buffer & start with a fresh one */
    /* this method works on all platforms, old & new */
    glBufferData(GL_ARRAY_BUFFER, imm_buffer_size, NULL, GL_DYNAMIC_DRAW);

    imm.buffer_offset = 0;
  }

  /*  printf("mapping %u to %u\n", imm.buffer_offset, imm.buffer_offset + bytes_needed - 1); */

  imm.buffer_data = (GLubyte *)glMapBufferRange(
      GL_ARRAY_BUFFER,
      imm.buffer_offset,
      bytes_needed,
      GL_MAP_WRITE_BIT | GL_MAP_UNSYNCHRONIZED_BIT |
          (imm.strict_vertex_len ? 0 : GL_MAP_FLUSH_EXPLICIT_BIT));

  imm.buffer_bytes_mapped = bytes_needed;
  imm.vertex_data = imm.buffer_data;
}

void immBeginAtMost(GPUPrimType prim_type, uint vertex_len)
{
  imm.strict_vertex_len = false;
  immBegin(prim_type, vertex_len);
}

static uint read_attr_location(const GPUAttrBinding *binding, uint a_idx)
{
  return (binding->loc_bits >> (4 * a_idx)) & 0xF;
}

static void immDrawSetup()
{
  /* set up VAO -- can be done during Begin or End really */
  glBindVertexArray(imm.vao_id);

  /* Enable/Disable vertex attributes as needed. */
  if (imm.attr_binding.enabled_bits != imm.prev_enabled_attr_bits) {
    for (uint loc = 0; loc < GPU_VERT_ATTR_MAX_LEN; loc++) {
      bool is_enabled = imm.attr_binding.enabled_bits & (1 << loc);
      bool was_enabled = imm.prev_enabled_attr_bits & (1 << loc);

      if (is_enabled && !was_enabled) {
        glEnableVertexAttribArray(loc);
      }
      else if (was_enabled && !is_enabled) {
        glDisableVertexAttribArray(loc);
      }
    }

    imm.prev_enabled_attr_bits = imm.attr_binding.enabled_bits;
  }

  const uint stride = imm.vertex_format.stride;

  for (uint a_idx = 0; a_idx < imm.vertex_format.attr_len; a_idx++) {
    const GPUVertAttr *a = &imm.vertex_format.attrs[a_idx];

    const uint offset = imm.buffer_offset + a->offset;
    const GLvoid *pointer = (const GLubyte *)0 + offset;

    const uint loc = read_attr_location(&imm.attr_binding, a_idx);

    switch (a->fetch_mode) {
      case GPU_FETCH_FLOAT:
      case GPU_FETCH_INT_TO_FLOAT:
        glVertexAttribPointer(loc, a->comp_len, a->gl_comp_type, GL_FALSE, stride, pointer);
        break;
      case GPU_FETCH_INT_TO_FLOAT_UNIT:
        glVertexAttribPointer(loc, a->comp_len, a->gl_comp_type, GL_TRUE, stride, pointer);
        break;
      case GPU_FETCH_INT:
        glVertexAttribIPointer(loc, a->comp_len, a->gl_comp_type, stride, pointer);
    }
  }
}

void immEnd()
{
  uint buffer_bytes_used;
  if (imm.strict_vertex_len) {
    buffer_bytes_used = imm.buffer_bytes_mapped;
  }
  else {
    if (imm.vertex_idx == imm.vertex_len) {
      buffer_bytes_used = imm.buffer_bytes_mapped;
    }
    else {
      imm.vertex_len = imm.vertex_idx;
      buffer_bytes_used = imm.vertex_format.vertexBufferSize(imm.vertex_len);
      /* unused buffer bytes are available to the next immBegin */
    }
    /* tell OpenGL what range was modified so it doesn't copy the whole mapped range */
    glFlushMappedBufferRange(GL_ARRAY_BUFFER, 0, buffer_bytes_used);
  }

  glUnmapBuffer(GL_ARRAY_BUFFER);

  if (imm.vertex_len > 0) {
    immDrawSetup();
#ifdef __APPLE__
    glDisable(GL_PRIMITIVE_RESTART);
#endif
    glDrawArrays(convert_prim_type_to_gl(imm.prim_type), 0, imm.vertex_len);
#ifdef __APPLE__
    glEnable(GL_PRIMITIVE_RESTART);
#endif
    /* These lines are causing crash on startup on some old GPU + drivers.
     * They are not required so just comment them. (T55722) */
    // glBindBuffer(GL_ARRAY_BUFFER, 0);
    // glBindVertexArray(0);
    /* prep for next immBegin */
    imm.buffer_offset += buffer_bytes_used;
  }

  /* prep for next immBegin */
  imm.prim_type = GPU_PRIM_NONE;
  imm.strict_vertex_len = true;
}

static void setAttrValueBit(uint attr_id)
{
  uint16_t mask = 1 << attr_id;
  imm.unassigned_attr_bits &= ~mask;
}

void immAttr1f(uint attr_id, float x)
{
  GPUVertAttr *attr = &imm.vertex_format.attrs[attr_id];
  setAttrValueBit(attr_id);

  float *data = (float *)(imm.vertex_data + attr->offset);
  /*  printf("%s %td %p\n", __FUNCTION__, (GLubyte*)data - imm.buffer_data, data); */

  data[0] = x;
}

void immAttr2f(uint attr_id, float x, float y)
{
  GPUVertAttr *attr = &imm.vertex_format.attrs[attr_id];
  setAttrValueBit(attr_id);

  float *data = (float *)(imm.vertex_data + attr->offset);
  /*  printf("%s %td %p\n", __FUNCTION__, (GLubyte*)data - imm.buffer_data, data); */

  data[0] = x;
  data[1] = y;
}

void immAttr3f(uint attr_id, float x, float y, float z)
{
  GPUVertAttr *attr = &imm.vertex_format.attrs[attr_id];
  setAttrValueBit(attr_id);

  float *data = (float *)(imm.vertex_data + attr->offset);
  /*  printf("%s %td %p\n", __FUNCTION__, (GLubyte*)data - imm.buffer_data, data); */

  data[0] = x;
  data[1] = y;
  data[2] = z;
}

void immAttr4f(uint attr_id, float x, float y, float z, float w)
{
  GPUVertAttr *attr = &imm.vertex_format.attrs[attr_id];
  setAttrValueBit(attr_id);

  float *data = (float *)(imm.vertex_data + attr->offset);
  /*  printf("%s %td %p\n", __FUNCTION__, (GLubyte*)data - imm.buffer_data, data); */

  data[0] = x;
  data[1] = y;
  data[2] = z;
  data[3] = w;
}

static void immEndVertex() /* and move on to the next vertex */
{
  /* Have all attributes been assigned values?
   * If not, copy value from previous vertex. */
  if (imm.unassigned_attr_bits) {
    for (uint a_idx = 0; a_idx < imm.vertex_format.attr_len; a_idx++) {
      if ((imm.unassigned_attr_bits >> a_idx) & 1) {
        const GPUVertAttr *a = &imm.vertex_format.attrs[a_idx];

        GLubyte *data = imm.vertex_data + a->offset;
        memcpy(data, data - imm.vertex_format.stride, a->sz);
      }
    }
  }

  imm.vertex_idx++;
  imm.vertex_data += imm.vertex_format.stride;
  imm.unassigned_attr_bits = imm.attr_binding.enabled_bits;
}

void immVertex2f(uint attr_id, float x, float y)
{
  immAttr2f(attr_id, x, y);
  immEndVertex();
}

void immVertex3f(uint attr_id, float x, float y, float z)
{
  immAttr3f(attr_id, x, y, z);
  immEndVertex();
}

void immVertex4f(uint attr_id, float x, float y, float z, float w)
{
  immAttr4f(attr_id, x, y, z, w);
  immEndVertex();
}
