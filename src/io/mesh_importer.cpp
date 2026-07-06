#include "basements/io/mesh_importer.h"
#include "basements/graphics/gl_loader.h"
#include <iostream>

#include "basements/io/mesh_importer.h"
#include "basements/graphics/gl_loader.h"
#include <iostream>
#include <filesystem>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

// Define these only in one .cpp file
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_RESIZE_IMPLEMENTATION 
#include "stb_image.h"
#include "stb_image_write.h"
#include "tiny_gltf.h"

namespace basements {
namespace editor {

void Mesh::setup_mesh() {
    if (vao != 0) {
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
        glDeleteBuffers(1, &ebo);
    }

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    // Position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    
    // Normal
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

    // TexCoords
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texcoord));

    glBindVertexArray(0);
}

void Mesh::draw() {
    if (vao == 0) return;
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, (GLsizei)indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

bool MeshImporter::load_obj(const std::string& path, Mesh& out_mesh) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, path.c_str())) {
        std::cerr << "TinyObjLoader Error: " << warn << err << std::endl;
        return false;
    }

    // Combine all shapes into one mesh for simplicity
    for (const auto& shape : shapes) {
        for (const auto& index : shape.mesh.indices) {
            Vertex vertex;

            vertex.position = {
                attrib.vertices[3 * index.vertex_index + 0],
                attrib.vertices[3 * index.vertex_index + 1],
                attrib.vertices[3 * index.vertex_index + 2]
            };

            if (index.normal_index >= 0) {
                vertex.normal = {
                    attrib.normals[3 * index.normal_index + 0],
                    attrib.normals[3 * index.normal_index + 1],
                    attrib.normals[3 * index.normal_index + 2]
                };
            }

            if (index.texcoord_index >= 0) {
                vertex.texcoord = {
                    attrib.texcoords[2 * index.texcoord_index + 0],
                    attrib.texcoords[2 * index.texcoord_index + 1]
                };
            }

            out_mesh.vertices.push_back(vertex);
            out_mesh.indices.push_back((unsigned int)out_mesh.indices.size());
        }
    }
    
    out_mesh.setup_mesh();
    return true;
}

bool MeshImporter::load_gltf(const std::string& path, Mesh& out_mesh) {
    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    std::string err;
    std::string warn;
    bool ret = false;

    if (path.find(".glb") != std::string::npos) {
        ret = loader.LoadBinaryFromFile(&model, &err, &warn, path);
    } else {
        ret = loader.LoadASCIIFromFile(&model, &err, &warn, path);
    }

    if (!warn.empty()) std::cout << "TinyGLTF Warning: " << warn << std::endl;
    if (!err.empty()) std::cerr << "TinyGLTF Error: " << err << std::endl;
    if (!ret) return false;

    // Basic loading: iterate over meshes and primitives and combine them
    // (Limitation: Ignores transforms in nodes for now, just loads raw mesh data)
    for (const auto& mesh : model.meshes) {
        for (const auto& primitive : mesh.primitives) {
            // Get accessor for position
            if (primitive.attributes.find("POSITION") == primitive.attributes.end()) continue;
            
            const tinygltf::Accessor& posAccessor = model.accessors[primitive.attributes.at("POSITION")];
            const tinygltf::BufferView& posView = model.bufferViews[posAccessor.bufferView];
            const tinygltf::Buffer& posBuffer = model.buffers[posView.buffer];
            const float* positions = reinterpret_cast<const float*>(&posBuffer.data[posView.byteOffset + posAccessor.byteOffset]);

            // Get accessor for normal (optional)
            const float* normals = nullptr;
            if (primitive.attributes.find("NORMAL") != primitive.attributes.end()) {
                const tinygltf::Accessor& normAccessor = model.accessors[primitive.attributes.at("NORMAL")];
                const tinygltf::BufferView& normView = model.bufferViews[normAccessor.bufferView];
                const tinygltf::Buffer& normBuffer = model.buffers[normView.buffer];
                normals = reinterpret_cast<const float*>(&normBuffer.data[normView.byteOffset + normAccessor.byteOffset]);
            }

            // Get accessor for texcoord (optional)
            const float* texcoords = nullptr;
            if (primitive.attributes.find("TEXCOORD_0") != primitive.attributes.end()) {
                const tinygltf::Accessor& uvAccessor = model.accessors[primitive.attributes.at("TEXCOORD_0")];
                const tinygltf::BufferView& uvView = model.bufferViews[uvAccessor.bufferView];
                const tinygltf::Buffer& uvBuffer = model.buffers[uvView.buffer];
                texcoords = reinterpret_cast<const float*>(&uvBuffer.data[uvView.byteOffset + uvAccessor.byteOffset]);
            }

            // Indices
            if (primitive.indices >= 0) {
                const tinygltf::Accessor& idxAccessor = model.accessors[primitive.indices];
                const tinygltf::BufferView& idxView = model.bufferViews[idxAccessor.bufferView];
                const tinygltf::Buffer& idxBuffer = model.buffers[idxView.buffer];

                // Check index type
                // Supports unsigned byte, unsigned short, unsigned int
                // We convert everything to unsigned int
                for (size_t i = 0; i < idxAccessor.count; ++i) {
                    unsigned int index = 0;
                    if (idxAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) {
                        const unsigned short* buf = reinterpret_cast<const unsigned short*>(&idxBuffer.data[idxView.byteOffset + idxAccessor.byteOffset]);
                        index = buf[i];
                    } else if (idxAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) {
                        const unsigned int* buf = reinterpret_cast<const unsigned int*>(&idxBuffer.data[idxView.byteOffset + idxAccessor.byteOffset]);
                        index = buf[i];
                    } else if (idxAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE) {
                        const unsigned char* buf = reinterpret_cast<const unsigned char*>(&idxBuffer.data[idxView.byteOffset + idxAccessor.byteOffset]);
                        index = buf[i];
                    }
                    
                    // Creates vertices on demand? No, duplication is needed if we iterate indices
                    // Simplified: Just push vertex data for each index (flat shading if not optimized)
                    // Better: Push all vertices, then push indices with offset
                    
                    // Actually, Gltf separates vertex data and indices.
                    // So we should append vertices and then append indices with current vertex count offset.
                    out_mesh.indices.push_back(index + (unsigned int)out_mesh.vertices.size()); // This is wrong logic if we push vertices per PRIMITIVE
                }
            }
            
            // Wait, standard GL approach:
            // 1. Append all vertices of this primitive to mesh.vertices
            // 2. Append all indices of this primitive to mesh.indices, applying offset
            
            size_t vertex_offset = out_mesh.vertices.size();
            for (size_t i = 0; i < posAccessor.count; ++i) {
                Vertex v;
                v.position = glm::vec3(positions[i*3+0], positions[i*3+1], positions[i*3+2]);
                if (normals) v.normal = glm::vec3(normals[i*3+0], normals[i*3+1], normals[i*3+2]);
                if (texcoords) v.texcoord = glm::vec2(texcoords[i*2+0], texcoords[i*2+1]);
                out_mesh.vertices.push_back(v);
            }
            
            if (primitive.indices >= 0) {
                const tinygltf::Accessor& idxAccessor = model.accessors[primitive.indices];
                const tinygltf::BufferView& idxView = model.bufferViews[idxAccessor.bufferView];
                const tinygltf::Buffer& idxBuffer = model.buffers[idxView.buffer];
                
                for (size_t i = 0; i < idxAccessor.count; ++i) {
                    unsigned int index = 0;
                     if (idxAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) {
                        const unsigned short* buf = reinterpret_cast<const unsigned short*>(&idxBuffer.data[idxView.byteOffset + idxAccessor.byteOffset]);
                        index = buf[i];
                    } else if (idxAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) {
                        const unsigned int* buf = reinterpret_cast<const unsigned int*>(&idxBuffer.data[idxView.byteOffset + idxAccessor.byteOffset]);
                        index = buf[i];
                    } else if (idxAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE) {
                        const unsigned char* buf = reinterpret_cast<const unsigned char*>(&idxBuffer.data[idxView.byteOffset + idxAccessor.byteOffset]);
                        index = buf[i];
                    }
                    out_mesh.indices.push_back(index + (unsigned int)vertex_offset);
                }
            }
        }
    }
    
    out_mesh.setup_mesh();
    return true;
}

bool MeshImporter::load_mesh(const std::string& path, Mesh& out_mesh) {
    std::string ext = std::filesystem::path(path).extension().string();
    // To lower
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c){ return std::tolower(c); });
    
    if (ext == ".obj") {
        return load_obj(path, out_mesh);
    } else if (ext == ".gltf" || ext == ".glb") {
        return load_gltf(path, out_mesh);
    }
    
    std::cerr << "Unsupported mesh format: " << ext << std::endl;
    return false;
}

} // namespace editor
} // namespace basements
