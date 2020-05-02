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

static uint padding(uint offset, uint alignment)
{
  const uint mod = offset % alignment;
  return (mod == 0) ? 0 : (alignment - mod);
}

uchar GPUVertFormat::copyAttributeName(const char *name)
{
  /* strncpy does 110% of what we need; let's do exactly 100% */
  uchar name_offset = this->name_offset;
  char *name_copy = this->names + name_offset;
  uint available = GPU_VERT_ATTR_NAMES_BUF_LEN - name_offset;
  bool terminated = false;

  for (uint i = 0; i < available; i++) {
    const char c = name[i];
    name_copy[i] = c;
    if (c == '\0') {
      terminated = true;
      this->name_offset += (i + 1);
      break;
    }
  }
#if TRUST_NO_ONE
  assert(terminated);
  assert(this->name_offset <= GPU_VERT_ATTR_NAMES_BUF_LEN);
#else
  (void)terminated;
#endif
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
#if TRUST_NO_ONE
  assert(this->name_len < GPU_VERT_FORMAT_MAX_NAMES); /* there's room for more */
  assert(this->attr_len < GPU_VERT_ATTR_MAX_LEN);     /* there's room for more */
  assert(!this->packed);                              /* packed means frozen/locked */
  assert((comp_len >= 1 && comp_len <= 4) || comp_len == 8 || comp_len == 12 || comp_len == 16);

  switch (comp_type) {
    case GPU_COMP_F32:
      /* float type can only kept as float */
      assert(fetch_mode == GPU_FETCH_FLOAT);
      break;
    case GPU_COMP_I10:
      /* 10_10_10 format intended for normals (xyz) or colors (rgb)
       * extra component packed.w can be manually set to { -2, -1, 0, 1 } */
      assert(comp_len == 3 || comp_len == 4);

      /* Not strictly required, may relax later. */
      assert(fetch_mode == GPU_FETCH_INT_TO_FLOAT_UNIT);

      break;
    default:
      /* integer types can be kept as int or converted/normalized to float */
      assert(fetch_mode != GPU_FETCH_FLOAT);
      /* only support float matrices (see Batch_update_program_bindings) */
      assert(comp_len != 8 && comp_len != 12 && comp_len != 16);
  }
#endif
  this->name_len++; /* multiname support */

  const uint attr_id = this->attr_len++;
  GPUVertAttr *attr = &this->attrs[attr_id];

  attr->names[attr->name_len++] = this->copyAttributeName(name);
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
#if TRUST_NO_ONE
  memset(this, 0, sizeof(GPUVertFormat));
#else
  attr_len = 0;
  packed = false;
  name_offset = 0;
  name_len = 0;

  for (uint i = 0; i < GPU_VERT_ATTR_MAX_LEN; i++) {
    attrs[i].name_len = 0;
  }
#endif
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

void GPUVertFormat::pack()
{
  /* For now, attributes are packed in the order they were added,
   * making sure each attribute is naturally aligned (add padding where necessary)
   * Later we can implement more efficient packing w/ reordering
   * (keep attribute ID order, adjust their offsets to reorder in buffer). */

  /* TODO: realloc just enough to hold the final combo string. And just enough to
   * hold used attributes, not all 16. */

  GPUVertAttr *a0 = &this->attrs[0];
  a0->offset = 0;
  uint offset = a0->sz;

  for (uint a_idx = 1; a_idx < this->attr_len; a_idx++) {
    GPUVertAttr *a = &this->attrs[a_idx];
    uint mid_padding = padding(offset, a->attrAlign());
    offset += mid_padding;
    a->offset = offset;
    offset += a->sz;
  }

  uint end_padding = padding(offset, a0->attrAlign());

  this->stride = offset + end_padding;
  this->packed = true;
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

static GLuint GPU_vao_alloc()
{
  GLuint new_vao_id = 0;
  glGenVertexArrays(1, &new_vao_id);
  return new_vao_id;
}

static void GPU_vao_free(GLuint vao_id)
{
  glDeleteVertexArrays(1, &vao_id);
}

GPUVertFormat *immVertexFormat()
{
  imm.vertex_format.clear();
  return &imm.vertex_format;
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

void immActivate()
{
#if TRUST_NO_ONE
  assert(initialized);
  assert(imm.prim_type == GPU_PRIM_NONE); /* make sure we're not between a Begin/End pair */
  assert(imm.vao_id == 0);
#endif
  imm.vao_id = GPU_vao_alloc();
}

void immDeactivate()
{
#if TRUST_NO_ONE
  assert(initialized);
  assert(imm.prim_type == GPU_PRIM_NONE); /* make sure we're not between a Begin/End pair */
  assert(imm.vao_id != 0);
#endif
  GPU_vao_free(imm.vao_id);
  imm.vao_id = 0;
  imm.prev_enabled_attr_bits = 0;
}

static void write_attr_location(GPUAttrBinding *binding, uint a_idx, uint location)
{
#if TRUST_NO_ONE
  assert(a_idx < GPU_VERT_ATTR_MAX_LEN);
  assert(location < GPU_VERT_ATTR_MAX_LEN);
#endif
  const uint shift = 4 * a_idx;
  const uint64_t mask = ((uint64_t)0xF) << shift;
  /* overwrite this attr's previous location */
  binding->loc_bits = (binding->loc_bits & ~mask) | (location << shift);
  /* mark this attr as enabled */
  binding->enabled_bits |= 1 << a_idx;
}

void immBegin(GPUPrimType prim_type, uint vertex_len, Shader *shader)
{
  if (!imm.vertex_format.packed) {
    imm.vertex_format.pack();
  }
  /* TODO(ish): need to get attribute locations and enable the correct
   * attributes */
  /* get_attr_locations(&imm.vertex_format, &imm.attr_binding, shaderface); */
  imm.attr_binding.clear();
  for (uint a_idx = 0; a_idx < imm.vertex_format.attr_len; a_idx++) {
    const GPUVertAttr *a = &imm.vertex_format.attrs[a_idx];
    for (uint n_idx = 0; n_idx < a->name_len; n_idx++) {
      const char *name = imm.vertex_format.getAttributeName(a, n_idx);

      uint location = glGetAttribLocation(shader->ID, name);
      write_attr_location(&imm.attr_binding, a_idx, location);
    }
  }

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
  const uint pre_padding = padding(imm.buffer_offset, imm.vertex_format.stride);

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

#if TRUST_NO_ONE
  assert(imm.buffer_data != NULL);
#endif

  imm.buffer_bytes_mapped = bytes_needed;
  imm.vertex_data = imm.buffer_data;
}

void immBeginAtMost(GPUPrimType prim_type, uint vertex_len, Shader *shader)
{
  imm.strict_vertex_len = false;
  immBegin(prim_type, vertex_len, shader);
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
        /* cout << "loc: " << loc << " a->comp_len: " << a->comp_len */
        /*      << " a->gl_comp_type: " << a->gl_comp_type << " stride: " << stride */
        /*      << " imm.buffer_offset: " << imm.buffer_offset << " a->offset: " << a->offset */
        /*      << " offset: " << offset << endl; */
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
#if TRUST_NO_ONE
  assert(imm.prim_type != GPU_PRIM_NONE); /* make sure we're between a Begin/End pair */
#endif
  uint buffer_bytes_used;
  if (imm.strict_vertex_len) {
#if TRUST_NO_ONE
    assert(imm.vertex_idx == imm.vertex_len); /* with all vertices defined */
#endif
    buffer_bytes_used = imm.buffer_bytes_mapped;
  }
  else {
#if TRUST_NO_ONE
    assert(imm.vertex_idx <= imm.vertex_len);
#endif
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
#if TRUST_NO_ONE
  assert(attr_id < imm.vertex_format.attr_len);
  assert(attr->comp_type == GPU_COMP_F32);
  assert(attr->comp_len == 1);
  assert(imm.vertex_idx < imm.vertex_len);
  assert(imm.prim_type != GPU_PRIM_NONE); /* make sure we're between a Begin/End pair */
#endif
  setAttrValueBit(attr_id);

  float *data = (float *)(imm.vertex_data + attr->offset);
  /*  printf("%s %td %p\n", __FUNCTION__, (GLubyte*)data - imm.buffer_data, data); */

  data[0] = x;
}

void immAttr2f(uint attr_id, float x, float y)
{
  GPUVertAttr *attr = &imm.vertex_format.attrs[attr_id];
#if TRUST_NO_ONE
  assert(attr_id < imm.vertex_format.attr_len);
  assert(attr->comp_type == GPU_COMP_F32);
  assert(attr->comp_len == 2);
  assert(imm.vertex_idx < imm.vertex_len);
  assert(imm.prim_type != GPU_PRIM_NONE); /* make sure we're between a Begin/End pair */
#endif
  setAttrValueBit(attr_id);

  float *data = (float *)(imm.vertex_data + attr->offset);
  /*  printf("%s %td %p\n", __FUNCTION__, (GLubyte*)data - imm.buffer_data, data); */

  data[0] = x;
  data[1] = y;
}

void immAttr3f(uint attr_id, float x, float y, float z)
{
  GPUVertAttr *attr = &imm.vertex_format.attrs[attr_id];
#if TRUST_NO_ONE
  assert(attr_id < imm.vertex_format.attr_len);
  assert(attr->comp_type == GPU_COMP_F32);
  assert(attr->comp_len == 3);
  assert(imm.vertex_idx < imm.vertex_len);
  assert(imm.prim_type != GPU_PRIM_NONE); /* make sure we're between a Begin/End pair */
#endif
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
#if TRUST_NO_ONE
  assert(attr_id < imm.vertex_format.attr_len);
  assert(attr->comp_type == GPU_COMP_F32);
  assert(attr->comp_len == 4);
  assert(imm.vertex_idx < imm.vertex_len);
  assert(imm.prim_type != GPU_PRIM_NONE); /* make sure we're between a Begin/End pair */
#endif
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
#if TRUST_NO_ONE
  assert(imm.prim_type != GPU_PRIM_NONE); /* make sure we're between a Begin/End pair */
  assert(imm.vertex_idx < imm.vertex_len);
#endif

  /* Have all attributes been assigned values?
   * If not, copy value from previous vertex. */
  if (imm.unassigned_attr_bits) {
#if TRUST_NO_ONE
    assert(imm.vertex_idx > 0); /* first vertex must have all attributes specified */
#endif
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
