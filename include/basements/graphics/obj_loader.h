#ifndef BASEMENTS_EDITOR_OBJ_LOADER_H
#define BASEMENTS_EDITOR_OBJ_LOADER_H

#include "basements/graphics/mesh.h"
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

namespace basements {
namespace editor {

class ObjLoader {
public:
    static bool load(const std::string& filepath, Mesh& out_mesh) {
        std::ifstream file(filepath);
        if (!file.is_open()) return false;

        std::vector<glm::vec3> temp_positions;
        std::vector<glm::vec3> temp_normals;
        std::vector<glm::vec2> temp_uvs;
        std::vector<Vertex> vertices;
        std::vector<unsigned int> indices;

        std::string line;
        while (std::getline(file, line)) {
            if (line.substr(0, 2) == "v ") {
                std::istringstream s(line.substr(2));
                glm::vec3 v; s >> v.x >> v.y >> v.z;
                temp_positions.push_back(v);
            } else if (line.substr(0, 3) == "vn ") {
                std::istringstream s(line.substr(3));
                glm::vec3 v; s >> v.x >> v.y >> v.z;
                temp_normals.push_back(v);
            } else if (line.substr(0, 3) == "vt ") {
                std::istringstream s(line.substr(3));
                glm::vec2 v; s >> v.x >> v.y;
                temp_uvs.push_back(v);
            } else if (line.substr(0, 2) == "f ") {
                // Simplified face parsing (assumes triangles)
                // Format: v/vt/vn
                std::string s1, s2, s3;
                std::istringstream s(line.substr(2));
                s >> s1 >> s2 >> s3;
                
                auto parse_vertex = [&](const std::string& str) {
                    std::stringstream ss(str);
                    std::string segment;
                    std::vector<std::string> seglist;
                    while(std::getline(ss, segment, '/')) {
                       seglist.push_back(segment);
                    }
                    
                    Vertex vertex;
                    if (seglist.size() > 0 && !seglist[0].empty()) 
                        vertex.position = temp_positions[std::stoi(seglist[0]) - 1];
                    
                    if (seglist.size() > 1 && !seglist[1].empty())
                        vertex.texcoord = temp_uvs[std::stoi(seglist[1]) - 1];
                    
                    if (seglist.size() > 2 && !seglist[2].empty())
                        vertex.normal = temp_normals[std::stoi(seglist[2]) - 1];
                        
                    return vertex;
                };

                vertices.push_back(parse_vertex(s1));
                indices.push_back(vertices.size() - 1);
                vertices.push_back(parse_vertex(s2));
                indices.push_back(vertices.size() - 1);
                vertices.push_back(parse_vertex(s3));
                indices.push_back(vertices.size() - 1);
                
                // If quad (s4), handle... skipping for MVP minimal
            }
        }
        
        out_mesh.vertices = vertices;
        out_mesh.indices = indices;
        out_mesh.setup_mesh();
        return true;
    }
};

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_OBJ_LOADER_H
