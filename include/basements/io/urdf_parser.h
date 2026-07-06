#pragma once
#include "basements/io/urdf_types.h"

// Forward declarations
namespace tinyxml2 {
    class XMLDocument;
    class XMLElement;
}

namespace basements {
namespace editor {
    class SceneGraph;
    using NodeID = uint64_t;
}
}

namespace basements {
namespace io {

/**
 * @brief Result of URDF parsing operation
 */
struct URDFParseResult {
    bool success = false;
    std::string error_message;
    URDFRobot robot;

    URDFParseResult() = default;
    URDFParseResult(bool success_, const std::string& error_ = "")
        : success(success_), error_message(error_) {}
};

/**
 * @brief URDF Parser - Import/Export robot descriptions
 * 
 * Supports:
 * - Parsing URDF XML files
 * - Converting URDF ↔ SceneGraph
 * - Exporting SceneGraph to URDF
 */
class URDFParser {
public:
    URDFParser() = default;

    // ========== Parsing ==========

    /**
     * @brief Parse URDF from file
     * @param filepath Path to .urdf file
     * @return Parse result with robot data or error
     */
    URDFParseResult parse_file(const std::string& filepath);

    /**
     * @brief Parse URDF from string
     * @param urdf_xml URDF XML content
     * @return Parse result with robot data or error
     */
    URDFParseResult parse_string(const std::string& urdf_xml);

    // ========== Conversion ==========
    // Note: SceneGraph conversion is available via urdf_converter.h (header-only)
    // Include that header in your code to use URDFConverter::to_scene_graph() and from_scene_graph()

    // ========== Export ==========

    /**
     * @brief Export URDF robot to file
     * @param robot URDF robot description
     * @param filepath Output file path
     * @return true if successful
     */
    bool export_to_file(const URDFRobot& robot,
                        const std::string& filepath);

    /**
     * @brief Export URDF robot to string
     * @param robot URDF robot description
     * @return URDF XML string
     */
    std::string export_to_string(const URDFRobot& robot);

    // ========== Utilities ==========

    /**
     * @brief Get last error message
     */
    const std::string& get_last_error() const { return last_error_; }

private:
    std::string last_error_;

    // Helper: Parse XML document
    URDFParseResult parse_document(class tinyxml2::XMLDocument& doc);

    // Helper: Parse individual elements
    URDFLink parse_link(class tinyxml2::XMLElement* elem);
    URDFJoint parse_joint(class tinyxml2::XMLElement* elem);
    URDFMaterial parse_material(class tinyxml2::XMLElement* elem);
    URDFGeometry parse_geometry(class tinyxml2::XMLElement* elem);
    URDFInertial parse_inertial(class tinyxml2::XMLElement* elem);
    URDFVisual parse_visual(class tinyxml2::XMLElement* elem);
    URDFCollision parse_collision(class tinyxml2::XMLElement* elem);
    URDFPose parse_pose(class tinyxml2::XMLElement* elem);
    URDFVector3 parse_vector3(const char* str);
    URDFQuaternion parse_quaternion(const char* rpy_str);  // Roll-Pitch-Yaw

    // Helper: Write XML elements
    void write_link(class tinyxml2::XMLElement* parent, const URDFLink& link);
    void write_joint(class tinyxml2::XMLElement* parent, const URDFJoint& joint);
    void write_material(class tinyxml2::XMLElement* parent, const URDFMaterial& material);
    void write_geometry(class tinyxml2::XMLElement* parent, const URDFGeometry& geometry);
    void write_inertial(class tinyxml2::XMLElement* parent, const URDFInertial& inertial);
    void write_visual(class tinyxml2::XMLElement* parent, const URDFVisual& visual);
    void write_collision(class tinyxml2::XMLElement* parent, const URDFCollision& collision);
    void write_pose(class tinyxml2::XMLElement* parent, const URDFPose& pose, const char* tag_name);
};

} // namespace io
} // namespace basements
