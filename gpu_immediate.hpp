#ifndef GPU_IMMEDIATE_HPP
#define GPU_IMMEDIATE_HPP

/* Based on Blender's gpu immediate work-alike system */

#include <glad/glad.h>
#include <cstring>
#include <cassert>
#include <iostream>

#include "shader.hpp"

using namespace std;

#define TRUST_NO_ONE 0

#define GPU_VERT_FORMAT_MAX_NAMES 63
#define GPU_VERT_ATTR_MAX_LEN 16
#define GPU_VERT_ATTR_MAX_NAMES 6
#define GPU_VERT_ATTR_NAMES_BUF_LEN 256

typedef unsigned int uint;
typedef unsigned char uchar;

enum GPUVertCompType {
  GPU_COMP_I8,
  GPU_COMP_U8,
  GPU_COMP_I16,
  GPU_COMP_U16,
  GPU_COMP_I32,
  GPU_COMP_U32,

  GPU_COMP_F32,

  GPU_COMP_I10,
};

enum GPUVertFetchMode {
  GPU_FETCH_FLOAT,
  GPU_FETCH_INT,
  GPU_FETCH_INT_TO_FLOAT_UNIT, /* 127 (ubyte) -> 0.5 (and so on for other int types) */
  GPU_FETCH_INT_TO_FLOAT,      /* 127 (any int type) -> 127.0 */
};

enum GPUPrimType {
  GPU_PRIM_POINTS,
  GPU_PRIM_LINES,
  GPU_PRIM_TRIS,
  GPU_PRIM_LINE_STRIP,
  GPU_PRIM_LINE_LOOP, /* GL has this, Vulkan does not */
  GPU_PRIM_TRI_STRIP,
  GPU_PRIM_TRI_FAN,

  GPU_PRIM_LINES_ADJ,
  GPU_PRIM_TRIS_ADJ,
  GPU_PRIM_LINE_STRIP_ADJ,

  GPU_PRIM_NONE,
};

class GPUAttrBinding {
 public:
  /** Store 4 bits for each of the 16 attributes. */
  uint64_t loc_bits;
  /** 1 bit for each attribute. */
  uint16_t enabled_bits;

  void clear()
  {
    loc_bits = 0;
    enabled_bits = 0;
  }
};

class GPUVertAttr {
 public:
  uint fetch_mode : 2;
  uint comp_type : 3;
  /* 1 to 4 or 8 or 12 or 16 */
  uint comp_len : 5;
  /* size in bytes, 1 to 64 */
  uint sz : 7;
  /* from beginning of vertex, in bytes */
  uint offset : 11;
  /* up to GPU_VERT_ATTR_MAX_NAMES */
  uint name_len : 3;
  uint gl_comp_type;
  /* -- 8 Bytes -- */
  uchar names[GPU_VERT_ATTR_MAX_NAMES];

  uint compSZ(GPUVertCompType type)
  {
    const GLubyte sizes[] = {1, 1, 2, 2, 4, 4, 4};
    return sizes[type];
  }

  uint attrSZ()
  {
    if (comp_type == GPU_COMP_I10) {
      return 4; /* always packed as 10_10_10_2 */
    }
    return comp_len * compSZ((GPUVertCompType)comp_type);
  }

  uint attrAlign()
  {
    if (comp_type == GPU_COMP_I10) {
      return 4; /* always packed as 10_10_10_2 */
    }
    uint c = compSZ((GPUVertCompType)comp_type);
    if (comp_len == 3 && c <= 2) {
      return 4 * c; /* AMD HW can't fetch these well, so pad it out (other vendors too?) */
    }
    else {
      return c; /* most fetches are ok if components are naturally aligned */
    }
  }
};

class GPUVertFormat {
 public:
  /** 0 to 16 (GPU_VERT_ATTR_MAX_LEN). */
  uint attr_len : 5;
  /** Total count of active vertex attribute names. (max GPU_VERT_FORMAT_MAX_NAMES) */
  uint name_len : 6;
  /** Stride in bytes, 1 to 1024. */
  uint stride : 11;
  /** Has the format been packed. */
  uint packed : 1;
  /** Current offset in names[]. */
  uint name_offset : 8;
  /** Store each attribute in one contiguous buffer region. */
  uint deinterleaved : 1;

  GPUVertAttr attrs[GPU_VERT_ATTR_MAX_LEN];
  char names[GPU_VERT_ATTR_NAMES_BUF_LEN];

  /* Functions */
  uchar copyAttributeName(const char *name);
  const char *getAttributeName(const GPUVertAttr *attr, uint n_idx);

  GPUVertFormat()
  {
    clear();
  }

  void clear();
  uint addAttribute(const char *name,
                    GPUVertCompType comp_type,
                    uint comp_len,
                    GPUVertFetchMode fetch_mode);
  int getAttributeID(const char *name);
  uint vertexBufferSize(uint vertex_len);
  void pack();
};

GPUVertFormat *immVertexFormat();
void immInit();
void immDestroy();

void immActivate();
void immDeactivate();

void immBegin(GPUPrimType prim_type, uint vertex_len, Shader *shader);
void immBeginAtMost(GPUPrimType prim_type, uint vertex_len, Shader *shader);
void immEnd();

/* Provide attribute values that can change per vertex. */
/* First vertex after immBegin must have all its attributes specified. */
/* Skipped attributes will continue using the previous value for that attr_id. */
void immAttr1f(uint attr_id, float x);
void immAttr2f(uint attr_id, float x, float y);
void immAttr3f(uint attr_id, float x, float y, float z);
void immAttr4f(uint attr_id, float x, float y, float z, float w);

/* Provide one last attribute value & end the current vertex. */
/* This is most often used for 2D or 3D position (similar to glVertex). */
void immVertex2f(uint attr_id, float x, float y);
void immVertex3f(uint attr_id, float x, float y, float z);
void immVertex4f(uint attr_id, float x, float y, float z, float w);

#endif
