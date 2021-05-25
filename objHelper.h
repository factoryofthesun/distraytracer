#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#include "tiny_obj_loader.h"

using namespace std;

void loadObj(string path, string mtl_path, vector<VEC3>& vertices, vector<VEC3I>& v_indices, vector<VEC2>& texcoords,
              vector<VEC3I>& t_indices, vector<VEC3>& normals, vector<VEC3I>& n_indices)
{
  tinyobj::ObjReaderConfig reader_config;
  reader_config.mtl_search_path = mtl_path; // Path to material files

  tinyobj::ObjReader reader;

  if (!reader.ParseFromFile(path, reader_config)) {
    if (!reader.Error().empty()) {
        cerr << "TinyObjReader: " << reader.Error();
    }
    exit(1);
  }

  if (!reader.Warning().empty()) {
    cout << "TinyObjReader: " << reader.Warning();
  }

  auto& attrib = reader.GetAttrib();
  auto& shapes = reader.GetShapes();
  auto& materials = reader.GetMaterials();

  // Build vector of vertices
  for (size_t i = 0; i < attrib.vertices.size()/3; i++)
  {
    float vx = attrib.vertices[3*i+0];
    float vy = attrib.vertices[3*i+1];
    float vz = attrib.vertices[3*i+2];
    vertices.push_back(VEC3(vx, vy, vz));
  }

  // Build vector of texcoords
  for (size_t i = 0; i < attrib.texcoords.size()/2; i++)
  {
    float vx = attrib.texcoords[2*i+0];
    float vy = attrib.texcoords[2*i+1];
    texcoords.push_back(VEC2(vx, vy));
  }

  // Build vector of normals
  for (size_t i = 0; i < attrib.normals.size()/3; i++)
  {
    float vx = attrib.vertices[3*i+0];
    float vy = attrib.vertices[3*i+1];
    float vz = attrib.vertices[3*i+2];
    normals.push_back(VEC3(vx, vy, vz));
  }

  // Loop over shapes
  for (size_t s = 0; s < shapes.size(); s++) {
    // Loop over faces(polygon)
    size_t index_offset = 0;
    for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
      size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

      // ASSUMPTION: 3 vertices per face (for triangles)
      int id0 = int(shapes[s].mesh.indices[index_offset].vertex_index);
      int id1 = int(shapes[s].mesh.indices[index_offset + 1].vertex_index);
      int id2 = int(shapes[s].mesh.indices[index_offset + 2].vertex_index);
      v_indices.push_back(VEC3I(id0, id1, id2));

      // Check if has texcoord index
      int tid0 = int(shapes[s].mesh.indices[index_offset].texcoord_index);
      int tid1 = int(shapes[s].mesh.indices[index_offset + 1].texcoord_index);
      int tid2 = int(shapes[s].mesh.indices[index_offset + 2].texcoord_index);
      t_indices.push_back(VEC3I(tid0, tid1, tid2));

      int nid0 = int(shapes[s].mesh.indices[index_offset].normal_index);
      int nid1 = int(shapes[s].mesh.indices[index_offset + 1].normal_index);
      int nid2 = int(shapes[s].mesh.indices[index_offset + 2].normal_index);
      n_indices.push_back(VEC3I(nid0, nid1, nid2));

      index_offset += fv;

      // per-face material
      shapes[s].mesh.material_ids[f];
    }
  }
}
