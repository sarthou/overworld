/*
The MIT License (MIT)

Copyright (c) 2012-Present, Syoyo Fujita and many contributors.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

//
// version 2.0.0 : Add new object oriented API. 1.x API is still provided.
//                 * Add python binding.
//                 * Support line primitive.
//                 * Support points primitive.
//                 * Support multiple search path for .mtl(v1 API).
//                 * Support vertex skinning weight `vw`(as an tinyobj
//                 extension). Note that this differs vertex weight([w]
//                 component in `v` line)
//                 * Support escaped whitespece in mtllib
//                 * Add robust triangulation using Mapbox
//                 earcut(TINYOBJLOADER_USE_MAPBOX_EARCUT).
// version 1.4.0 : Modifed parseTextureName API
// version 1.3.1 : Make parseTextureName API public
// version 1.3.0 : Separate warning and error message(breaking API of LoadObj)
// version 1.2.3 : Added color space extension('-colorspace') to tex opts.
// version 1.2.2 : Parse multiple group names.
// version 1.2.1 : Added initial support for line('l') primitive(PR #178)
// version 1.2.0 : Hardened implementation(#175)
// version 1.1.1 : Support smoothing groups(#162)
// version 1.1.0 : Support parsing vertex color(#144)
// version 1.0.8 : Fix parsing `g` tag just after `usemtl`(#138)
// version 1.0.7 : Support multiple tex options(#126)
// version 1.0.6 : Add TINYOBJLOADER_USE_DOUBLE option(#124)
// version 1.0.5 : Ignore `Tr` when `d` exists in MTL(#43)
// version 1.0.4 : Support multiple filenames for 'mtllib'(#112)
// version 1.0.3 : Support parsing texture options(#85)
// version 1.0.2 : Improve parsing speed by about a factor of 2 for large
// files(#105)
// version 1.0.1 : Fixes a shape is lost if obj ends with a 'usemtl'(#104)
// version 1.0.0 : Change data structure. Change license from BSD to MIT.
//

//
// Use this in *one* .cc
//   #define TINYOBJLOADER_IMPLEMENTATION
//   #include "tiny_obj_loader.h"
//

#ifndef TINY_OBJ_LOADER_H_
#define TINY_OBJ_LOADER_H_

#include <map>
#include <string>
#include <vector>

namespace tinyobj {

  struct Material_t
  {
    std::string name;

    double ambient[3];
    double diffuse[3];
    double specular[3];
    double shininess;

    std::string ambient_texname;  // map_Ka. For ambient or ambient occlusion.
    std::string diffuse_texname;  // map_Kd
    std::string specular_texname; // map_Ks
    std::string normal_texname;   // norm. For normal mapping.
  };

  struct tag_t
  {
    std::string name;

    std::vector<int> intValues;
    std::vector<double> floatValues;
    std::vector<std::string> stringValues;
  };

  // Index struct to support different indices for vtx/normal/texcoord.
  // -1 means not used.
  struct Index_t
  {
    int vertex_index;
    int normal_index;
    int texcoord_index;
  };

  struct Mesh_t
  {
    std::string name;
    std::vector<Index_t> indices;
    std::vector<unsigned int>
      num_face_vertices;                           // The number of vertices per
                                                   // face. 3 = triangle, 4 = quad, ...
    std::vector<int> material_ids;                 // per-face material ID
    std::vector<unsigned int> smoothing_group_ids; // per-face smoothing group
                                                   // ID(0 = off. positive value
                                                   // = group id)
    std::vector<tag_t> tags;                       // SubD tag
  };

  // Vertex attributes
  struct Attrib_t
  {
    std::vector<double> vertices; // 'v'(xyz)

    // For backward compatibility, we store vertex weight in separate array.
    std::vector<double> normals;   // 'vn'
    std::vector<double> texcoords; // 'vt'(uv)

    Attrib_t() {}
  };

  class MaterialReader
  {
  public:
    MaterialReader() {}
    virtual ~MaterialReader();

    virtual bool operator()(const std::string& matId,
                            std::vector<Material_t>* materials,
                            std::map<std::string, int>* matMap, std::string* warn,
                            std::string* err) = 0;
  };

  ///
  /// Read .mtl from a file.
  ///
  class MaterialFileReader : public MaterialReader
  {
  public:
    // Path could contain separator(';' in Windows, ':' in Posix)
    explicit MaterialFileReader(const std::string& mtl_basedir)
      : m_mtlBaseDir(mtl_basedir) {}
    virtual ~MaterialFileReader() override {}
    virtual bool operator()(const std::string& matId,
                            std::vector<Material_t>* materials,
                            std::map<std::string, int>* matMap, std::string* warn,
                            std::string* err) override;

  private:
    std::string m_mtlBaseDir;
  };

  ///
  /// Read .mtl from a stream.
  ///
  class MaterialStreamReader : public MaterialReader
  {
  public:
    explicit MaterialStreamReader(std::istream& inStream)
      : m_inStream(inStream) {}
    virtual ~MaterialStreamReader() override {}
    virtual bool operator()(const std::string& matId,
                            std::vector<Material_t>* materials,
                            std::map<std::string, int>* matMap, std::string* warn,
                            std::string* err) override;

  private:
    std::istream& m_inStream;
  };

  // v2 API

  ///
  /// Wavefront .obj reader class(v2 API)
  ///
  class ObjReader
  {
  public:
    ObjReader() : valid_(false) {}

    ///
    /// Load .obj and .mtl from a file.
    ///
    /// @param[in] filename wavefront .obj filename
    /// @param[in] mtl_search_path Reader configuration
    ///
    bool parseFromFile(const std::string& filename,
                       const std::string& mtl_search_path = "");

    ///
    /// Parse .obj from a text string.
    /// Need to supply .mtl text string by `mtl_text`.
    /// This function ignores `mtllib` line in .obj text.
    ///
    /// @param[in] obj_text wavefront .obj filename
    /// @param[in] mtl_text wavefront .mtl filename
    ///
    bool ParseFromString(const std::string& obj_text, const std::string& mtl_text);

    ///
    /// .obj was loaded or parsed correctly.
    ///
    bool isValid() const { return valid_; }

    const Attrib_t& getAttrib() const { return attrib_; }

    const std::vector<Mesh_t>& getMeshes() const { return shapes_; }

    const std::vector<Material_t>& getMaterials() const { return materials_; }

    ///
    /// Warning message(may be filled after `Load` or `Parse`)
    ///
    const std::string& Warning() const { return warning_; }

    ///
    /// Error message(filled when `Load` or `Parse` failed)
    ///
    const std::string& Error() const { return error_; }

  private:
    bool valid_;

    Attrib_t attrib_;
    std::vector<Mesh_t> shapes_;
    std::vector<Material_t> materials_;

    std::string warning_;
    std::string error_;
  };

  /// ==>>========= Legacy v1 API =============================================

  /// Loads .obj from a file.
  /// 'attrib', 'shapes' and 'materials' will be filled with parsed shape data
  /// 'shapes' will be filled with parsed shape data
  /// Returns true when loading .obj become success.
  /// Returns warning message into `warn`, and error message into `err`
  /// 'mtl_basedir' is optional, and used for base directory for .mtl file.
  /// In default(`nullptr'), .mtl file is searched from an application's working
  /// directory.
  /// Option 'default_vcols_fallback' specifies whether vertex colors should
  /// always be defined, even if no colors are given (fallback to white).
  bool LoadObj(Attrib_t* attrib, std::vector<Mesh_t>* shapes,
               std::vector<Material_t>* materials, std::string* warn,
               std::string* err, const char* filename,
               const char* mtl_basedir = nullptr);

  /// Loads object from a std::istream, uses `readMatFn` to retrieve
  /// std::istream for materials.
  /// Returns true when loading .obj become success.
  /// Returns warning and error message into `err`
  bool LoadObj(Attrib_t* attrib, std::vector<Mesh_t>* shapes,
               std::vector<Material_t>* materials, std::string* warn,
               std::string* err, std::istream* inStream,
               MaterialReader* readMatFn = nullptr);

  /// Loads materials into std::map
  void LoadMtl(std::map<std::string, int>* material_map,
               std::vector<Material_t>* materials, std::istream* inStream,
               std::string* warning, std::string* err);

  ///
  /// Parse texture name and texture option for custom texture parameter through
  /// material::unknown_parameter
  ///
  /// @param[out] texname Parsed texture name
  /// @param[in] linebuf Input string
  ///
  bool parseTextureName(std::string& texture_name, const char* linebuf);

  /// =<<========== Legacy v1 API =============================================

} // namespace tinyobj

#endif // TINY_OBJ_LOADER_H_
