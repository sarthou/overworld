#include "overworld/Engine/Common/Models/Loaders/TinyObjReader.h"

#include <cassert>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace tinyobj {

  MaterialReader::~MaterialReader() {}

  struct vertex_index_t
  {
    int v_idx, vt_idx, vn_idx;
    vertex_index_t() : v_idx(-1), vt_idx(-1), vn_idx(-1) {}
    explicit vertex_index_t(int idx) : v_idx(idx), vt_idx(idx), vn_idx(idx) {}
    vertex_index_t(int vidx, int vtidx, int vnidx)
      : v_idx(vidx), vt_idx(vtidx), vn_idx(vnidx) {}
  };

  struct TagSizes_t
  {
    TagSizes_t() : num_ints(0), num_reals(0), num_strings(0) {}
    int num_ints;
    int num_reals;
    int num_strings;
  };

  // See
  // http://stackoverflow.com/questions/6089231/getting-std-ifstream-to-handle-lf-cr-and-crlf
  static std::istream& safeGetline(std::istream& is, std::string& t)
  {
    t.clear();

    // The characters in the stream are read one-by-one using a std::streambuf.
    // That is faster than reading them one-by-one using the std::istream.
    // Code that uses streambuf this way must be guarded by a sentry object.
    // The sentry object performs various tasks,
    // such as thread synchronization and updating the stream state.

    std::istream::sentry se(is, true);
    std::streambuf* sb = is.rdbuf();

    if(se)
    {
      for(;;)
      {
        int c = sb->sbumpc();
        switch(c)
        {
        case '\n':
          return is;
        case '\r':
          if(sb->sgetc() == '\n')
            sb->sbumpc();
          return is;
        case EOF:
          // Also handle the case when the last line has no line ending
          if(t.empty())
            is.setstate(std::ios::eofbit);
          return is;
        default:
          t += static_cast<char>(c);
        }
      }
    }

    return is;
  }

#define IS_SPACE(x) (((x) == ' ') || ((x) == '\t'))
#define IS_DIGIT(x) \
  (static_cast<unsigned int>((x) - '0') < static_cast<unsigned int>(10))
#define IS_NEW_LINE(x) (((x) == '\r') || ((x) == '\n') || ((x) == '\0'))

  template<typename T>
  static inline std::string toString(const T& t)
  {
    std::stringstream ss;
    ss << t;
    return ss.str();
  }

  struct warning_context
  {
    std::string* warn;
    size_t line_number;
  };

  // Make index zero-base, and also support relative index.
  static inline bool fixIndex(int idx, int n, int* ret, bool allow_zero,
                              const warning_context& context)
  {
    if(!ret)
    {
      return false;
    }

    if(idx > 0)
    {
      (*ret) = idx - 1;
      return true;
    }

    if(idx == 0)
    {
      // zero is not allowed according to the spec.
      if(context.warn)
      {
        (*context.warn) +=
          "A zero value index found (will have a value of -1 for normal and "
          "tex indices. Line " +
          toString(context.line_number) + ").\n";
      }

      (*ret) = idx - 1;
      return allow_zero;
    }

    if(idx < 0)
    {
      (*ret) = n + idx; // negative value = relative
      if((*ret) < 0)
      {
        return false; // invalid relative index
      }
      return true;
    }

    return false; // never reach here.
  }

  static inline std::string parseString(const char** token)
  {
    std::string s;
    (*token) += strspn((*token), " \t");
    size_t e = strcspn((*token), " \t\r");
    s = std::string((*token), &(*token)[e]);
    (*token) += e;
    return s;
  }

  static inline int parseInt(const char** token)
  {
    (*token) += strspn((*token), " \t");
    int i = atoi((*token));
    (*token) += strcspn((*token), " \t\r");
    return i;
  }

  static inline double parseReal(const char** token, double default_value = 0.0)
  {
    (*token) += strspn((*token), " \t");
    const char* end = (*token) + strcspn((*token), " \t\r");
    double val = default_value;
    if(std::from_chars((*token), end, val).ec != std::errc{})
      val = default_value;
    (*token) = end;
    return val;
  }

  static inline void parseReal2(double* x, double* y, const char** token,
                                const double default_x = 0.0,
                                const double default_y = 0.0)
  {
    (*x) = parseReal(token, default_x);
    (*y) = parseReal(token, default_y);
  }

  static inline void parseReal3(double* x, double* y, double* z,
                                const char** token, const double default_x = 0.0,
                                const double default_y = 0.0,
                                const double default_z = 0.0)
  {
    (*x) = parseReal(token, default_x);
    (*y) = parseReal(token, default_y);
    (*z) = parseReal(token, default_z);
  }

  static TagSizes_t parseTagTriple(const char** token)
  {
    TagSizes_t ts;

    (*token) += strspn((*token), " \t");
    ts.num_ints = atoi((*token));
    (*token) += strcspn((*token), "/ \t\r");
    if((*token)[0] != '/')
      return ts;

    (*token)++; // Skip '/'

    (*token) += strspn((*token), " \t");
    ts.num_reals = atoi((*token));
    (*token) += strcspn((*token), "/ \t\r");
    if((*token)[0] != '/')
      return ts;

    (*token)++; // Skip '/'

    ts.num_strings = parseInt(token);

    return ts;
  }

  // Parse triples with index offsets: i, i/j/k, i//k, i/j
  static bool parseTriple(const char** token, int vsize, int vnsize, int vtsize,
                          vertex_index_t* ret, const warning_context& context)
  {
    if(!ret)
    {
      return false;
    }

    vertex_index_t vi(-1);

    if(!fixIndex(atoi((*token)), vsize, &vi.v_idx, false, context))
    {
      return false;
    }

    (*token) += strcspn((*token), "/ \t\r");
    if((*token)[0] != '/')
    {
      (*ret) = vi;
      return true;
    }
    (*token)++;

    // i//k
    if((*token)[0] == '/')
    {
      (*token)++;
      if(!fixIndex(atoi((*token)), vnsize, &vi.vn_idx, true, context))
      {
        return false;
      }
      (*token) += strcspn((*token), "/ \t\r");
      (*ret) = vi;
      return true;
    }

    // i/j/k or i/j
    if(!fixIndex(atoi((*token)), vtsize, &vi.vt_idx, true, context))
    {
      return false;
    }

    (*token) += strcspn((*token), "/ \t\r");
    if((*token)[0] != '/')
    {
      (*ret) = vi;
      return true;
    }

    // i/j/k
    (*token)++; // skip '/'
    if(!fixIndex(atoi((*token)), vnsize, &vi.vn_idx, true, context))
    {
      return false;
    }
    (*token) += strcspn((*token), "/ \t\r");

    (*ret) = vi;

    return true;
  }

  bool parseTextureName(std::string& texture_name, const char* linebuf)
  {
    bool found_texname = false;

    const char* token = linebuf; // Assume line ends with nullptr

    while(!IS_NEW_LINE((*token)))
    {
      token += strspn(token, " \t"); // skip space
      if((0 == strncmp(token, "-blendu", 7)) && IS_SPACE((token[7])))
        continue;
      else if((0 == strncmp(token, "-blendv", 7)) && IS_SPACE((token[7])))
        continue;
      else if((0 == strncmp(token, "-clamp", 6)) && IS_SPACE((token[6])))
        continue;
      else if((0 == strncmp(token, "-boost", 6)) && IS_SPACE((token[6])))
        continue;
      else if((0 == strncmp(token, "-bm", 3)) && IS_SPACE((token[3])))
        continue;
      else if((0 == strncmp(token, "-o", 2)) && IS_SPACE((token[2])))
        continue;
      else if((0 == strncmp(token, "-s", 2)) && IS_SPACE((token[2])))
        continue;
      else if((0 == strncmp(token, "-t", 2)) && IS_SPACE((token[2])))
        continue;
      else if((0 == strncmp(token, "-type", 5)) && IS_SPACE((token[5])))
        continue;
      else if((0 == strncmp(token, "-texres", 7)) && IS_SPACE((token[7])))
        continue;
      else if((0 == strncmp(token, "-imfchan", 8)) && IS_SPACE((token[8])))
        continue;
      else if((0 == strncmp(token, "-mm", 3)) && IS_SPACE((token[3])))
        continue;
      else if((0 == strncmp(token, "-colorspace", 11)) && IS_SPACE((token[11])))
        continue;
      else
      {
        // Read filename until line end to parse filename containing whitespace
        texture_name = std::string(token);
        token += texture_name.length();

        found_texname = true;
      }
    }

    return found_texname;
  }

  static void initMaterial(Material_t* material)
  {
    material->name = "";
    material->ambient_texname = "";
    material->diffuse_texname = "";
    material->specular_texname = "";

    material->has_ambient = false;
    material->has_diffuse = false;
    material->has_specular = false;

    for(int i = 0; i < 3; i++)
    {
      material->ambient[i] = static_cast<double>(0.0);
      material->diffuse[i] = static_cast<double>(0.0);
      material->specular[i] = static_cast<double>(0.0);
    }
    material->shininess = static_cast<double>(-1.0);

    material->normal_texname = "";
  }

  // code from https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
  template<typename T>
  static int pnpoly(int nvert, T* vertx, T* verty, T testx, T testy)
  {
    int i, j, c = 0;
    for(i = 0, j = nvert - 1; i < nvert; j = i++)
    {
      if(((verty[i] > testy) != (verty[j] > testy)) &&
         (testx <
          (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) +
            vertx[i]))
        c = !c;
    }
    return c;
  }

  // TODO(syoyo): refactor function.
  static bool exportGroupsToShape(Mesh_t* shape, const std::vector<std::vector<vertex_index_t>>& face_group,
                                  const std::vector<tag_t>& tags,
                                  const int material_id, const std::string& name,
                                  const std::vector<double>& v, std::string* warn)
  {
    shape->name = name;
    shape->material_id = material_id;

    // polygon
    if(face_group.empty() == false)
    {
      // Flatten vertices and indices
      for(size_t i = 0; i < face_group.size(); i++)
      {
        const std::vector<vertex_index_t>& face = face_group[i];

        size_t npolys = face.size();

        if(npolys < 3)
        {
          // Face must have 3+ vertices.
          if(warn)
          {
            (*warn) += "Degenerated face found\n.";
          }
          continue;
        }

        if(npolys != 3)
        {
          if(npolys == 4)
          {
            vertex_index_t i0 = face[0];
            vertex_index_t i1 = face[1];
            vertex_index_t i2 = face[2];
            vertex_index_t i3 = face[3];

            size_t vi0 = size_t(i0.v_idx);
            size_t vi1 = size_t(i1.v_idx);
            size_t vi2 = size_t(i2.v_idx);
            size_t vi3 = size_t(i3.v_idx);

            if(((3 * vi0 + 2) >= v.size()) || ((3 * vi1 + 2) >= v.size()) ||
               ((3 * vi2 + 2) >= v.size()) || ((3 * vi3 + 2) >= v.size()))
            {
              // Invalid triangle.
              // FIXME(syoyo): Is it ok to simply skip this invalid triangle?
              if(warn)
              {
                (*warn) += "Face with invalid vertex index found.\n";
              }
              continue;
            }

            double v0x = v[vi0 * 3 + 0];
            double v0y = v[vi0 * 3 + 1];
            double v0z = v[vi0 * 3 + 2];
            double v1x = v[vi1 * 3 + 0];
            double v1y = v[vi1 * 3 + 1];
            double v1z = v[vi1 * 3 + 2];
            double v2x = v[vi2 * 3 + 0];
            double v2y = v[vi2 * 3 + 1];
            double v2z = v[vi2 * 3 + 2];
            double v3x = v[vi3 * 3 + 0];
            double v3y = v[vi3 * 3 + 1];
            double v3z = v[vi3 * 3 + 2];

            // There are two candidates to split the quad into two triangles.
            //
            // Choose the shortest edge.
            // TODO: Is it better to determine the edge to split by calculating
            // the area of each triangle?
            //
            // +---+
            // |\  |
            // | \ |
            // |  \|
            // +---+
            //
            // +---+
            // |  /|
            // | / |
            // |/  |
            // +---+

            double e02x = v2x - v0x;
            double e02y = v2y - v0y;
            double e02z = v2z - v0z;
            double e13x = v3x - v1x;
            double e13y = v3y - v1y;
            double e13z = v3z - v1z;

            double sqr02 = e02x * e02x + e02y * e02y + e02z * e02z;
            double sqr13 = e13x * e13x + e13y * e13y + e13z * e13z;

            Index_t idx0, idx1, idx2, idx3;

            idx0.vertex_index = i0.v_idx;
            idx0.normal_index = i0.vn_idx;
            idx0.texcoord_index = i0.vt_idx;
            idx1.vertex_index = i1.v_idx;
            idx1.normal_index = i1.vn_idx;
            idx1.texcoord_index = i1.vt_idx;
            idx2.vertex_index = i2.v_idx;
            idx2.normal_index = i2.vn_idx;
            idx2.texcoord_index = i2.vt_idx;
            idx3.vertex_index = i3.v_idx;
            idx3.normal_index = i3.vn_idx;
            idx3.texcoord_index = i3.vt_idx;

            if(sqr02 < sqr13)
            {
              // [0, 1, 2], [0, 2, 3]
              shape->indices.push_back(idx0);
              shape->indices.push_back(idx1);
              shape->indices.push_back(idx2);

              shape->indices.push_back(idx0);
              shape->indices.push_back(idx2);
              shape->indices.push_back(idx3);
            }
            else
            {
              // [0, 1, 3], [1, 2, 3]
              shape->indices.push_back(idx0);
              shape->indices.push_back(idx1);
              shape->indices.push_back(idx3);

              shape->indices.push_back(idx1);
              shape->indices.push_back(idx2);
              shape->indices.push_back(idx3);
            }
          }
          else
          {
            vertex_index_t i0 = face[0];
            vertex_index_t i1(-1);
            vertex_index_t i2 = face[1];

            // find the two axes to work in
            size_t axes[2] = {1, 2};
            for(size_t k = 0; k < npolys; ++k)
            {
              i0 = face[(k + 0) % npolys];
              i1 = face[(k + 1) % npolys];
              i2 = face[(k + 2) % npolys];
              size_t vi0 = size_t(i0.v_idx);
              size_t vi1 = size_t(i1.v_idx);
              size_t vi2 = size_t(i2.v_idx);

              if(((3 * vi0 + 2) >= v.size()) || ((3 * vi1 + 2) >= v.size()) ||
                 ((3 * vi2 + 2) >= v.size()))
              {
                // Invalid triangle.
                // FIXME(syoyo): Is it ok to simply skip this invalid triangle?
                continue;
              }
              double v0x = v[vi0 * 3 + 0];
              double v0y = v[vi0 * 3 + 1];
              double v0z = v[vi0 * 3 + 2];
              double v1x = v[vi1 * 3 + 0];
              double v1y = v[vi1 * 3 + 1];
              double v1z = v[vi1 * 3 + 2];
              double v2x = v[vi2 * 3 + 0];
              double v2y = v[vi2 * 3 + 1];
              double v2z = v[vi2 * 3 + 2];
              double e0x = v1x - v0x;
              double e0y = v1y - v0y;
              double e0z = v1z - v0z;
              double e1x = v2x - v1x;
              double e1y = v2y - v1y;
              double e1z = v2z - v1z;
              double cx = std::fabs(e0y * e1z - e0z * e1y);
              double cy = std::fabs(e0z * e1x - e0x * e1z);
              double cz = std::fabs(e0x * e1y - e0y * e1x);
              const double epsilon = std::numeric_limits<double>::epsilon();

              if(cx > epsilon || cy > epsilon || cz > epsilon)
              {
                // found a corner
                if(cx > cy && cx > cz)
                {}
                else
                {
                  axes[0] = 0;
                  if(cz > cx && cz > cy)
                    axes[1] = 1;
                }
                break;
              }
            }

            std::vector<vertex_index_t> remaining_face = face; // copy
            size_t guess_vert = 0;
            vertex_index_t ind[3];
            double vx[3];
            double vy[3];

            // How many iterations can we do without decreasing the remaining
            // vertices.
            size_t remaining_iterations = face.size();
            size_t previous_remaining_vertices =
              remaining_face.size();

            while(remaining_face.size() > 3 &&
                  remaining_iterations > 0)
            {
              npolys = remaining_face.size();
              if(guess_vert >= npolys)
              {
                guess_vert -= npolys;
              }

              if(previous_remaining_vertices != npolys)
              {
                // The number of remaining vertices decreased. Reset counters.
                previous_remaining_vertices = npolys;
                remaining_iterations = npolys;
              }
              else
              {
                // We didn't consume a vertex on previous iteration, reduce the
                // available iterations.
                remaining_iterations--;
              }

              for(size_t k = 0; k < 3; k++)
              {
                ind[k] = remaining_face[(guess_vert + k) % npolys];
                size_t vi = size_t(ind[k].v_idx);
                if(((vi * 3 + axes[0]) >= v.size()) ||
                   ((vi * 3 + axes[1]) >= v.size()))
                {
                  // ???
                  vx[k] = 0.0;
                  vy[k] = 0.0;
                }
                else
                {
                  vx[k] = v[vi * 3 + axes[0]];
                  vy[k] = v[vi * 3 + axes[1]];
                }
              }

              //
              // area is calculated per face
              //
              double e0x = vx[1] - vx[0];
              double e0y = vy[1] - vy[0];
              double e1x = vx[2] - vx[1];
              double e1y = vy[2] - vy[1];
              double cross = e0x * e1y - e0y * e1x;

              double area =
                (vx[0] * vy[1] - vy[0] * vx[1]) * static_cast<double>(0.5);
              // if an internal angle
              if(cross * area < static_cast<double>(0.0))
              {
                guess_vert += 1;
                continue;
              }

              // check all other verts in case they are inside this triangle
              bool overlap = false;
              for(size_t otherVert = 3; otherVert < npolys; ++otherVert)
              {
                size_t idx = (guess_vert + otherVert) % npolys;

                if(idx >= remaining_face.size())
                  continue;

                size_t ovi = size_t(remaining_face[idx].v_idx);

                if(((ovi * 3 + axes[0]) >= v.size()) ||
                   ((ovi * 3 + axes[1]) >= v.size()))
                  continue;

                double tx = v[ovi * 3 + axes[0]];
                double ty = v[ovi * 3 + axes[1]];
                if(pnpoly(3, vx, vy, tx, ty))
                {
                  overlap = true;
                  break;
                }
              }

              if(overlap)
              {
                guess_vert += 1;
                continue;
              }

              // this triangle is an ear
              {
                Index_t idx0, idx1, idx2;
                idx0.vertex_index = ind[0].v_idx;
                idx0.normal_index = ind[0].vn_idx;
                idx0.texcoord_index = ind[0].vt_idx;
                idx1.vertex_index = ind[1].v_idx;
                idx1.normal_index = ind[1].vn_idx;
                idx1.texcoord_index = ind[1].vt_idx;
                idx2.vertex_index = ind[2].v_idx;
                idx2.normal_index = ind[2].vn_idx;
                idx2.texcoord_index = ind[2].vt_idx;

                shape->indices.push_back(idx0);
                shape->indices.push_back(idx1);
                shape->indices.push_back(idx2);
              }

              // remove v1 from the list
              size_t removed_vert_index = (guess_vert + 1) % npolys;
              while(removed_vert_index + 1 < npolys)
              {
                remaining_face[removed_vert_index] =
                  remaining_face[removed_vert_index + 1];
                removed_vert_index += 1;
              }
              remaining_face.pop_back();
            }

            // remaining_face.size() << "\n";
            if(remaining_face.size() == 3)
            {
              i0 = remaining_face[0];
              i1 = remaining_face[1];
              i2 = remaining_face[2];
              {
                Index_t idx0, idx1, idx2;
                idx0.vertex_index = i0.v_idx;
                idx0.normal_index = i0.vn_idx;
                idx0.texcoord_index = i0.vt_idx;
                idx1.vertex_index = i1.v_idx;
                idx1.normal_index = i1.vn_idx;
                idx1.texcoord_index = i1.vt_idx;
                idx2.vertex_index = i2.v_idx;
                idx2.normal_index = i2.vn_idx;
                idx2.texcoord_index = i2.vt_idx;

                shape->indices.push_back(idx0);
                shape->indices.push_back(idx1);
                shape->indices.push_back(idx2);
              }
            }
          } // npolys
        }
        else
        {
          for(size_t k = 0; k < npolys; k++)
          {
            Index_t idx;
            idx.vertex_index = face[k].v_idx;
            idx.normal_index = face[k].vn_idx;
            idx.texcoord_index = face[k].vt_idx;
            shape->indices.push_back(idx);
          }
        }
      }

      shape->tags = tags;
    }

    return true;
  }

  // Split a string with specified delimiter character and escape character.
  // https://rosettacode.org/wiki/Tokenize_a_string_with_escaping#C.2B.2B
  static void SplitString(const std::string& s, char delim, char escape,
                          std::vector<std::string>& elems)
  {
    std::string token;

    bool escaping = false;
    for(size_t i = 0; i < s.size(); ++i)
    {
      char ch = s[i];
      if(escaping)
      {
        escaping = false;
      }
      else if(ch == escape)
      {
        escaping = true;
        continue;
      }
      else if(ch == delim)
      {
        if(!token.empty())
        {
          elems.push_back(token);
        }
        token.clear();
        continue;
      }
      token += ch;
    }

    elems.push_back(token);
  }

  static std::string JoinPath(const std::string& dir,
                              const std::string& filename)
  {
    if(dir.empty())
    {
      return filename;
    }
    else
    {
      // check '/'
      char lastChar = *dir.rbegin();
      if(lastChar != '/')
      {
        return dir + std::string("/") + filename;
      }
      else
      {
        return dir + filename;
      }
    }
  }

  void LoadMtl(std::map<std::string, int>* material_map,
               std::vector<Material_t>* materials, std::istream* inStream,
               const std::string& texture_path,
               std::string* warning, std::string* err)
  {
    (void)err;

    // Create a default material anyway.
    Material_t material;
    initMaterial(&material);

    // has_kd is used to set a default diffuse value when map_Kd is present
    // and Kd is not.
    bool has_kd = false;

    std::stringstream warn_ss;

    size_t line_no = 0;
    std::string linebuf;
    while(inStream->peek() != -1)
    {
      safeGetline(*inStream, linebuf);
      line_no++;

      // Trim trailing whitespace.
      if(linebuf.size() > 0)
      {
        linebuf = linebuf.substr(0, linebuf.find_last_not_of(" \t") + 1);
      }

      // Trim newline '\r\n' or '\n'
      if(linebuf.size() > 0)
      {
        if(linebuf[linebuf.size() - 1] == '\n')
          linebuf.erase(linebuf.size() - 1);
      }
      if(linebuf.size() > 0)
      {
        if(linebuf[linebuf.size() - 1] == '\r')
          linebuf.erase(linebuf.size() - 1);
      }

      // Skip if empty line.
      if(linebuf.empty())
      {
        continue;
      }

      // Skip leading space.
      const char* token = linebuf.c_str();
      token += strspn(token, " \t");

      assert(token);
      if(token[0] == '\0')
        continue; // empty line

      if(token[0] == '#')
        continue; // comment line

      // new mtl
      if((0 == strncmp(token, "newmtl", 6)) && IS_SPACE((token[6])))
      {
        // flush previous material.
        if(!material.name.empty())
        {
          material_map->insert(std::pair<std::string, int>(
            material.name, static_cast<int>(materials->size())));
          materials->push_back(material);
        }

        // initial temporary material
        initMaterial(&material);

        // set new mtl name
        token += 7;
        {
          std::string namebuf = parseString(&token);
          // TODO: empty name check?
          if(namebuf.empty())
          {
            if(warning)
            {
              (*warning) += "empty material name in `newmtl`\n";
            }
          }
          material.name = namebuf;
        }
        continue;
      }

      if(token[0] == 'K' && IS_SPACE((token[2])))
      {
        // ambient
        if(token[1] == 'a')
        {
          token += 2;
          double r, g, b;
          parseReal3(&r, &g, &b, &token);
          material.ambient[0] = r;
          material.ambient[1] = g;
          material.ambient[2] = b;
          material.has_ambient = true;
          continue;
        }

        // diffuse
        if(token[1] == 'd')
        {
          token += 2;
          double r, g, b;
          parseReal3(&r, &g, &b, &token);
          material.diffuse[0] = r;
          material.diffuse[1] = g;
          material.diffuse[2] = b;
          material.has_diffuse = true;
          has_kd = true;
          continue;
        }

        // specular
        if(token[1] == 's')
        {
          token += 2;
          double r, g, b;
          parseReal3(&r, &g, &b, &token);
          material.specular[0] = r;
          material.specular[1] = g;
          material.specular[2] = b;
          material.has_specular = true;
          continue;
        }
      }

      // transmittance
      // if((token[0] == 'K' && token[1] == 't' && IS_SPACE((token[2]))) ||
      //   (token[0] == 'T' && token[1] == 'f' && IS_SPACE((token[2]))))
      //  continue;

      // ior(index of refraction)
      // if(token[0] == 'N' && token[1] == 'i' && IS_SPACE((token[2])))
      //  continue;

      // emission
      // if(token[0] == 'K' && token[1] == 'e' && IS_SPACE(token[2]))
      //  continue;

      // shininess
      if(token[0] == 'N' && token[1] == 's' && IS_SPACE(token[2]))
      {
        token += 2;
        material.shininess = parseReal(&token);
        continue;
      }

      // illum model
      // if(0 == strncmp(token, "illum", 5) && IS_SPACE(token[5]))
      //  continue;

      // dissolve
      // if((token[0] == 'd' && IS_SPACE(token[1])))
      //  continue;
      // if(token[0] == 'T' && token[1] == 'r' && IS_SPACE(token[2]))
      //  continue;

      // PBR: roughness
      // if(token[0] == 'P' && token[1] == 'r' && IS_SPACE(token[2]))
      //  continue;

      // PBR: metallic
      // if(token[0] == 'P' && token[1] == 'm' && IS_SPACE(token[2]))
      //  continue;

      // PBR: sheen
      // if(token[0] == 'P' && token[1] == 's' && IS_SPACE(token[2]))
      //  continue;

      // PBR: clearcoat thickness
      // if(token[0] == 'P' && token[1] == 'c' && IS_SPACE(token[2]))
      //  continue;

      // PBR: clearcoat roughness
      // if((0 == strncmp(token, "Pcr", 3)) && IS_SPACE(token[3]))
      //  continue;

      // PBR: anisotropy
      // if((0 == strncmp(token, "aniso", 5)) && IS_SPACE(token[5]))
      //  continue;

      // PBR: anisotropy rotation
      // if((0 == strncmp(token, "anisor", 6)) && IS_SPACE(token[6]))
      //  continue;

      // ambient or ambient occlusion texture
      if((0 == strncmp(token, "map_Ka", 6)) && IS_SPACE(token[6]))
      {
        token += 7;
        parseTextureName(material.ambient_texname, token);
        if(material.ambient_texname.find('/') == std::string::npos)
          material.ambient_texname = texture_path + "/" + material.ambient_texname;
        continue;
      }

      // diffuse texture
      if((0 == strncmp(token, "map_Kd", 6)) && IS_SPACE(token[6]))
      {
        token += 7;
        parseTextureName(material.diffuse_texname, token);
        if(material.diffuse_texname.find('/') == std::string::npos)
          material.diffuse_texname = texture_path + "/" + material.diffuse_texname;

        // Set a decent diffuse default value if a diffuse texture is specified
        // without a matching Kd value.
        if(!has_kd)
        {
          material.diffuse[0] = static_cast<double>(0.6);
          material.diffuse[1] = static_cast<double>(0.6);
          material.diffuse[2] = static_cast<double>(0.6);
        }

        continue;
      }

      // specular texture
      if((0 == strncmp(token, "map_Ks", 6)) && IS_SPACE(token[6]))
      {
        token += 7;
        parseTextureName(material.specular_texname, token);
        if(material.specular_texname.find('/') == std::string::npos)
          material.specular_texname = texture_path + "/" + material.specular_texname;
        continue;
      }

      // specular highlight texture
      // if((0 == strncmp(token, "map_Ns", 6)) && IS_SPACE(token[6]))
      //  continue;

      // bump texture
      // if((0 == strncmp(token, "map_bump", 8)) && IS_SPACE(token[8]))
      //  continue;

      // bump texture
      // if((0 == strncmp(token, "bump", 4)) && IS_SPACE(token[4]))
      //  continue;

      // alpha texture
      // if((0 == strncmp(token, "map_d", 5)) && IS_SPACE(token[5]))
      //  continue;

      // displacement texture
      // if((0 == strncmp(token, "map_disp", 8)) && IS_SPACE(token[8]))
      //  continue;

      // displacement texture
      // if((0 == strncmp(token, "disp", 4)) && IS_SPACE(token[4]))
      //  continue;

      // reflection map
      // if((0 == strncmp(token, "refl", 4)) && IS_SPACE(token[4]))
      //  continue;

      // PBR: roughness texture
      // if((0 == strncmp(token, "map_Pr", 6)) && IS_SPACE(token[6]))
      //  continue;

      // PBR: metallic texture
      // if((0 == strncmp(token, "map_Pm", 6)) && IS_SPACE(token[6]))
      //  continue;

      // PBR: sheen texture
      // if((0 == strncmp(token, "map_Ps", 6)) && IS_SPACE(token[6]))
      //  continue;

      // PBR: emissive texture
      // if((0 == strncmp(token, "map_Ke", 6)) && IS_SPACE(token[6]))
      //  continue;

      // PBR: normal map texture
      if((0 == strncmp(token, "norm", 4)) && IS_SPACE(token[4]))
      {
        token += 5;
        parseTextureName(material.normal_texname, token);
        if(material.normal_texname.find('/') == std::string::npos)
          material.normal_texname = texture_path + "/" + material.normal_texname;
        continue;
      }
    }
    // flush last material.
    material_map->insert(std::pair<std::string, int>(material.name, static_cast<int>(materials->size())));
    materials->push_back(material);

    if(warning)
    {
      (*warning) = warn_ss.str();
    }
  }

  bool MaterialFileReader::operator()(const std::string& matId,
                                      std::vector<Material_t>* materials,
                                      std::map<std::string, int>* matMap,
                                      std::string* warn, std::string* err)
  {
    if(m_mtlBaseDir.empty() == false)
    {
      // https://stackoverflow.com/questions/5167625/splitting-a-c-stdstring-using-tokens-e-g
      std::vector<std::string> paths;
      std::istringstream f(m_mtlBaseDir);

      std::string s;
      while(getline(f, s, ':'))
      {
        paths.push_back(s);
      }

      for(size_t i = 0; i < paths.size(); i++)
      {
        std::string filepath = JoinPath(paths[i], matId);

        std::ifstream matIStream(filepath.c_str());
        if(matIStream)
        {
          LoadMtl(matMap, materials, &matIStream, m_mtlBaseDir, warn, err);

          return true;
        }
      }

      std::stringstream ss;
      ss << "Material file [ " << matId
         << " ] not found in a path : " << m_mtlBaseDir << "\n";
      if(warn)
      {
        (*warn) += ss.str();
      }
      return false;
    }
    else
    {
      std::string filepath = matId;
      std::ifstream matIStream(filepath.c_str());
      if(matIStream)
      {
        LoadMtl(matMap, materials, &matIStream, m_mtlBaseDir, warn, err);

        return true;
      }

      std::stringstream ss;
      ss << "Material file [ " << filepath
         << " ] not found in a path : " << m_mtlBaseDir << "\n";
      if(warn)
      {
        (*warn) += ss.str();
      }

      return false;
    }
  }

  bool MaterialStreamReader::operator()(const std::string& matId,
                                        std::vector<Material_t>* materials,
                                        std::map<std::string, int>* matMap,
                                        std::string* warn, std::string* err)
  {
    (void)err;
    (void)matId;
    if(!m_inStream)
    {
      std::stringstream ss;
      ss << "Material stream in error state. \n";
      if(warn)
      {
        (*warn) += ss.str();
      }
      return false;
    }

    LoadMtl(matMap, materials, &m_inStream, "", warn, err);

    return true;
  }

  bool LoadObj(Attrib_t* attrib, std::vector<Mesh_t>* shapes,
               std::vector<Material_t>* materials, std::string* warn,
               std::string* err, const char* filename, const char* mtl_basedir)
  {
    attrib->vertices.clear();
    attrib->normals.clear();
    attrib->texcoords.clear();
    shapes->clear();

    std::stringstream errss;

    std::ifstream ifs(filename);
    if(!ifs)
    {
      errss << "Cannot open file [" << filename << "]\n";
      if(err)
      {
        (*err) = errss.str();
      }
      return false;
    }

    std::string baseDir = mtl_basedir ? mtl_basedir : "";
    if(!baseDir.empty())
    {
      if(baseDir[baseDir.length() - 1] != '/')
        baseDir += '/';
    }
    MaterialFileReader matFileReader(baseDir);

    return LoadObj(attrib, shapes, materials, warn, err, &ifs, &matFileReader);
  }

  bool LoadObj(Attrib_t* attrib, std::vector<Mesh_t>* shapes,
               std::vector<Material_t>* materials, std::string* warn,
               std::string* err, std::istream* inStream,
               MaterialReader* readMatFn /*= nullptr*/)
  {
    std::stringstream errss;

    std::vector<double> v;
    std::vector<double> vn;
    std::vector<double> vt;
    std::vector<tag_t> tags;
    std::vector<std::vector<vertex_index_t>> face_group;
    std::string name;

    // material
    std::set<std::string> material_filenames;
    std::map<std::string, int> material_map;
    int material = -1;

    int greatest_v_idx = -1;
    int greatest_vn_idx = -1;
    int greatest_vt_idx = -1;

    Mesh_t shape;

    size_t line_num = 0;
    std::string linebuf;
    while(inStream->peek() != -1)
    {
      safeGetline(*inStream, linebuf);

      line_num++;

      // Trim newline '\r\n' or '\n'
      if(linebuf.size() > 0)
      {
        if(linebuf[linebuf.size() - 1] == '\n')
          linebuf.erase(linebuf.size() - 1);
      }
      if(linebuf.size() > 0)
      {
        if(linebuf[linebuf.size() - 1] == '\r')
          linebuf.erase(linebuf.size() - 1);
      }

      // Skip if empty line.
      if(linebuf.empty())
      {
        continue;
      }

      // Skip leading space.
      const char* token = linebuf.c_str();
      token += strspn(token, " \t");

      assert(token);
      if(token[0] == '\0')
        continue; // empty line

      if(token[0] == '#')
        continue; // comment line

      // vertex
      if(token[0] == 'v' && IS_SPACE((token[1])))
      {
        token += 2;
        double x, y, z;

        parseReal3(&x, &y, &z, &token); // TODO

        v.push_back(x);
        v.push_back(y);
        v.push_back(z);

        continue;
      }

      // normal
      if(token[0] == 'v' && token[1] == 'n' && IS_SPACE((token[2])))
      {
        token += 3;
        double x, y, z;
        parseReal3(&x, &y, &z, &token);
        vn.push_back(x);
        vn.push_back(y);
        vn.push_back(z);
        continue;
      }

      // texcoord
      if(token[0] == 'v' && token[1] == 't' && IS_SPACE((token[2])))
      {
        token += 3;
        double x, y;
        parseReal2(&x, &y, &token);
        vt.push_back(x);
        vt.push_back(y);
        continue;
      }

      // skin weight. tinyobj extension
      // if(token[0] == 'v' && token[1] == 'w' && IS_SPACE((token[2])))
      //  continue;

      warning_context context;
      context.warn = warn;
      context.line_number = line_num;

      // line
      // if(token[0] == 'l' && IS_SPACE((token[1])))
      //  continue;

      // points
      // if(token[0] == 'p' && IS_SPACE((token[1])))
      //  continue;

      // face
      if(token[0] == 'f' && IS_SPACE((token[1])))
      {
        token += 2;
        token += strspn(token, " \t");

        std::vector<vertex_index_t> face;

        face.reserve(3);

        while(!IS_NEW_LINE(token[0]))
        {
          vertex_index_t vi;
          if(!parseTriple(&token, static_cast<int>(v.size() / 3),
                          static_cast<int>(vn.size() / 3),
                          static_cast<int>(vt.size() / 2), &vi, context))
          {
            if(err)
            {
              (*err) +=
                "Failed to parse `f' line (e.g. a zero value for vertex index "
                "or invalid relative vertex index). Line " +
                toString(line_num) + ").\n";
            }
            return false;
          }

          greatest_v_idx = greatest_v_idx > vi.v_idx ? greatest_v_idx : vi.v_idx;
          greatest_vn_idx =
            greatest_vn_idx > vi.vn_idx ? greatest_vn_idx : vi.vn_idx;
          greatest_vt_idx =
            greatest_vt_idx > vi.vt_idx ? greatest_vt_idx : vi.vt_idx;

          face.push_back(vi);
          size_t n = strspn(token, " \t\r");
          token += n;
        }

        // replace with emplace_back + std::move on C++11
        face_group.push_back(face);

        continue;
      }

      // use mtl
      if((0 == strncmp(token, "usemtl", 6)))
      {
        token += 6;
        std::string namebuf = parseString(&token);

        int newMaterialId = -1;
        std::map<std::string, int>::const_iterator it =
          material_map.find(namebuf);
        if(it != material_map.end())
        {
          newMaterialId = it->second;
        }
        else
        {
          // { error!! material not found }
          if(warn)
          {
            (*warn) += "material [ '" + namebuf + "' ] not found in .mtl\n";
          }
        }

        if(newMaterialId != material)
        {
          // Create per-face material. Thus we don't add `shape` to `shapes` at
          // this time.
          // just clear `faceGroup` after `exportGroupsToShape()` call.
          exportGroupsToShape(&shape, face_group, tags, material, name,
                              v, warn);
          face_group.clear();
          material = newMaterialId;
        }

        continue;
      }

      // load mtl
      if((0 == strncmp(token, "mtllib", 6)) && IS_SPACE((token[6])))
      {
        if(readMatFn != nullptr)
        {
          token += 7;

          std::vector<std::string> filenames;
          SplitString(std::string(token), ' ', '\\', filenames);

          if(filenames.empty())
          {
            if(warn)
            {
              std::stringstream ss;
              ss << "Looks like empty filename for mtllib. Use default "
                    "material (line "
                 << line_num << ".)\n";

              (*warn) += ss.str();
            }
          }
          else
          {
            bool found = false;
            for(size_t s = 0; s < filenames.size(); s++)
            {
              if(material_filenames.count(filenames[s]) > 0)
              {
                found = true;
                continue;
              }

              std::string warn_mtl;
              std::string err_mtl;
              bool ok = (*readMatFn)(filenames[s].c_str(), materials,
                                     &material_map, &warn_mtl, &err_mtl);
              if(warn && (!warn_mtl.empty()))
              {
                (*warn) += warn_mtl;
              }

              if(err && (!err_mtl.empty()))
              {
                (*err) += err_mtl;
              }

              if(ok)
              {
                found = true;
                material_filenames.insert(filenames[s]);
                break;
              }
            }

            if(!found)
            {
              if(warn)
              {
                (*warn) +=
                  "Failed to load material file(s). Use default "
                  "material.\n";
              }
            }
          }
        }

        continue;
      }

      // group name
      if(token[0] == 'g' && IS_SPACE((token[1])))
      {
        // flush previous face group.
        bool ret = exportGroupsToShape(&shape, face_group, tags, material, name,
                                       v, warn);
        (void)ret; // return value not used.

        if(shape.indices.size() > 0)
          shapes->push_back(shape);

        shape = Mesh_t();

        material = -1;
        face_group.clear();

        std::vector<std::string> names;

        while(!IS_NEW_LINE(token[0]))
        {
          std::string str = parseString(&token);
          names.push_back(str);
          token += strspn(token, " \t\r"); // skip tag
        }

        // names[0] must be 'g'

        if(names.size() < 2)
        {
          // 'g' with empty names
          if(warn)
          {
            std::stringstream ss;
            ss << "Empty group name. line: " << line_num << "\n";
            (*warn) += ss.str();
            name = "";
          }
        }
        else
        {
          std::stringstream ss;
          ss << names[1];

          // tinyobjloader does not support multiple groups for a primitive.
          // Currently we concatinate multiple group names with a space to get
          // single group name.

          for(size_t i = 2; i < names.size(); i++)
            ss << " " << names[i];

          name = ss.str();
        }

        continue;
      }

      // object name
      if(token[0] == 'o' && IS_SPACE((token[1])))
      {
        // flush previous face group.
        bool ret = exportGroupsToShape(&shape, face_group, tags, material, name,
                                       v, warn);
        (void)ret; // return value not used.

        if(shape.indices.size() > 0)
          shapes->push_back(shape);

        material = -1;
        face_group.clear();
        shape = Mesh_t();

        // @todo { multiple object name? }
        token += 2;
        std::stringstream ss;
        ss << token;
        name = ss.str();

        continue;
      }

      if(token[0] == 't' && IS_SPACE(token[1]))
      {
        const int max_tag_nums = 8192; // FIXME(syoyo): Parameterize.
        tag_t tag;

        token += 2;

        tag.name = parseString(&token);

        TagSizes_t ts = parseTagTriple(&token);

        if(ts.num_ints < 0)
        {
          ts.num_ints = 0;
        }
        if(ts.num_ints > max_tag_nums)
        {
          ts.num_ints = max_tag_nums;
        }

        if(ts.num_reals < 0)
        {
          ts.num_reals = 0;
        }
        if(ts.num_reals > max_tag_nums)
        {
          ts.num_reals = max_tag_nums;
        }

        if(ts.num_strings < 0)
        {
          ts.num_strings = 0;
        }
        if(ts.num_strings > max_tag_nums)
        {
          ts.num_strings = max_tag_nums;
        }

        tag.intValues.resize(static_cast<size_t>(ts.num_ints));

        for(size_t i = 0; i < static_cast<size_t>(ts.num_ints); ++i)
        {
          tag.intValues[i] = parseInt(&token);
        }

        tag.floatValues.resize(static_cast<size_t>(ts.num_reals));
        for(size_t i = 0; i < static_cast<size_t>(ts.num_reals); ++i)
        {
          tag.floatValues[i] = parseReal(&token);
        }

        tag.stringValues.resize(static_cast<size_t>(ts.num_strings));
        for(size_t i = 0; i < static_cast<size_t>(ts.num_strings); ++i)
        {
          tag.stringValues[i] = parseString(&token);
        }

        tags.push_back(tag);

        continue;
      }

      // smoothing group id
      // if(token[0] == 's' && IS_SPACE(token[1]))
      //  continue;

      // Ignore unknown command.
    }

    if(greatest_v_idx >= static_cast<int>(v.size() / 3))
    {
      if(warn)
      {
        std::stringstream ss;
        ss << "Vertex indices out of bounds (line " << line_num << ".)\n\n";
        (*warn) += ss.str();
      }
    }
    if(greatest_vn_idx >= static_cast<int>(vn.size() / 3))
    {
      if(warn)
      {
        std::stringstream ss;
        ss << "Vertex normal indices out of bounds (line " << line_num
           << ".)\n\n";
        (*warn) += ss.str();
      }
    }
    if(greatest_vt_idx >= static_cast<int>(vt.size() / 2))
    {
      if(warn)
      {
        std::stringstream ss;
        ss << "Vertex texcoord indices out of bounds (line " << line_num
           << ".)\n\n";
        (*warn) += ss.str();
      }
    }

    bool ret = exportGroupsToShape(&shape, face_group, tags, material, name, v, warn);
    // exportGroupsToShape return false when `usemtl` is called in the last
    // line.
    // we also add `shape` to `shapes` when `shape.mesh` has already some
    // faces(indices)
    if(ret || shape.indices
                .size())
    { // FIXME(syoyo): Support other prims(e.g. lines)
      shapes->push_back(shape);
    }
    face_group.clear(); // for safety

    if(err)
    {
      (*err) += errss.str();
    }

    attrib->vertices.swap(v);
    attrib->normals.swap(vn);
    attrib->texcoords.swap(vt);

    return true;
  }

  bool ObjReader::parseFromFile(const std::string& filename,
                                const std::string& mtl_search_path)
  {
    std::string search_path;

    if(mtl_search_path.empty())
    {
      //
      // split at last '/'(for unixish system) or '\\'(for windows) to get
      // the base directory of .obj file
      //
      size_t pos = filename.find_last_of("/\\");
      if(pos != std::string::npos)
      {
        search_path = filename.substr(0, pos);
      }
    }
    else
    {
      search_path = mtl_search_path;
    }

    valid_ = LoadObj(&attrib_, &shapes_, &materials_, &warning_, &error_,
                     filename.c_str(), search_path.c_str());

    return valid_;
  }

  bool ObjReader::ParseFromString(const std::string& obj_text,
                                  const std::string& mtl_text)
  {
    std::stringbuf obj_buf(obj_text);
    std::stringbuf mtl_buf(mtl_text);

    std::istream obj_ifs(&obj_buf);
    std::istream mtl_ifs(&mtl_buf);

    MaterialStreamReader mtl_ss(mtl_ifs);

    valid_ = LoadObj(&attrib_, &shapes_, &materials_, &warning_, &error_,
                     &obj_ifs, &mtl_ss);

    return valid_;
  }

} // namespace tinyobj
