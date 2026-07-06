#ifndef BASEMENTS_EDITOR_SCENE_SERIALIZER_H
#define BASEMENTS_EDITOR_SCENE_SERIALIZER_H

/**
 * @file scene_serializer.h
 * @brief Scene file I/O for multiple formats
 * 
 * Supported formats:
 *   - .bscene (JSON) - Native format, full feature support
 *   - .urdf - Robot model import
 *   - .obj / .stl - Mesh import
 *   - .gltf / .glb - Universal 3D format
 */

#include "basements/engine/scene_graph.h"
#include <string>
#include <memory>
#include <vector>
#include <functional>

namespace basements {
namespace editor {
namespace io {

// ============================================================================
// File Format Detection
// ============================================================================

enum class FileFormat {
    UNKNOWN,
    BSCENE,     // Native JSON format
    URDF,       // Robot description
    OBJ,        // Wavefront OBJ
    STL,        // STereoLithography
    GLTF,       // glTF 2.0 (JSON)
    GLB,        // glTF 2.0 (Binary)
    FBX         // Autodesk FBX (future)
};

/**
 * @brief Detect file format from extension
 */
FileFormat detect_format(const std::string& path);

// ============================================================================
// Scene Serializer (Native Format)
// ============================================================================

/**
 * @brief Save/Load scenes in native .bscene format (JSON)
 */
class SceneSerializer {
public:
    /**
     * @brief Save scene to file
     * @param scene Scene graph to save
     * @param path Output file path (.bscene)
     * @return true if successful
     */
    static bool save(const SceneGraph& scene, const std::string& path);
    
    /**
     * @brief Load scene from file
     * @param path Input file path (.bscene)
     * @param out_scene Output scene graph
     * @return true if successful
     */
    static bool load(const std::string& path, SceneGraph& out_scene);
    
    /**
     * @brief Export scene to JSON string
     */
    static std::string to_json(const SceneGraph& scene, bool pretty = true);
    
    /**
     * @brief Import scene from JSON string
     */
    static bool from_json(const std::string& json, SceneGraph& out_scene);
};

// ============================================================================
// URDF Importer
// ============================================================================

struct URDFImportOptions {
    float scale             = 1.0f;     // Scale factor
    bool import_joints      = true;     // Import joint constraints
    bool import_visuals     = true;     // Import visual meshes
    bool import_collisions  = true;     // Import collision shapes
    bool merge_fixed_links  = false;    // Merge fixed joint links
};

/**
 * @brief Import robot models from URDF files
 */
class URDFImporter {
public:
    static bool import(const std::string& path, SceneGraph& out_scene, 
                       const URDFImportOptions& options = URDFImportOptions());
    
    static bool can_import(const std::string& path);
};

// ============================================================================
// Mesh Importer (OBJ/STL)
// ============================================================================

struct MeshImportOptions {
    float scale             = 1.0f;
    bool auto_center        = true;     // Center mesh at origin
    bool generate_collision = true;     // Generate convex hull collision
    bool split_by_material  = false;    // Create separate bodies per material
};

/**
 * @brief Import 3D meshes from OBJ/STL files
 */
class MeshImporter {
public:
    static bool import_obj(const std::string& path, SceneGraph& out_scene,
                           const MeshImportOptions& options = MeshImportOptions());
    
    static bool import_stl(const std::string& path, SceneGraph& out_scene,
                           const MeshImportOptions& options = MeshImportOptions());
};

// ============================================================================
// glTF Importer
// ============================================================================

struct GLTFImportOptions {
    float scale             = 1.0f;
    bool import_animations  = false;    // Animation support (future)
    bool import_materials   = true;
    bool import_cameras     = false;
    bool import_lights      = false;
};

/**
 * @brief Import scenes from glTF/GLB files
 */
class GLTFImporter {
public:
    static bool import(const std::string& path, SceneGraph& out_scene,
                       const GLTFImportOptions& options = GLTFImportOptions());
};

// ============================================================================
// Code Exporter (Generate C++ code)
// ============================================================================

/**
 * @brief Export scene as C++ code for programmatic use
 * 
 * Generates code like:
 * @code
 *   PhysicsWorldGPU world;
 *   BodyHandle ground = world.create_body(make_box_body({0,0,0}, {50,0.1,50}, 0));
 *   BodyHandle box1 = world.create_body(make_box_body({0,5,0}, {1,1,1}, 1));
 * @endcode
 */
class CodeExporter {
public:
    static std::string generate_cpp(const SceneGraph& scene);
    static bool export_cpp(const SceneGraph& scene, const std::string& path);
};

// ============================================================================
// Universal Import Function
// ============================================================================

/**
 * @brief Import any supported file format
 * 
 * Auto-detects format and calls appropriate importer.
 */
bool import_file(const std::string& path, SceneGraph& out_scene);

/**
 * @brief Get list of supported import extensions
 */
std::vector<std::string> get_supported_import_extensions();

/**
 * @brief Get file dialog filter string
 * 
 * Returns: "Scene Files (*.bscene *.urdf *.obj *.stl *.gltf *.glb)"
 */
std::string get_import_filter_string();

} // namespace io
} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_SCENE_SERIALIZER_H
