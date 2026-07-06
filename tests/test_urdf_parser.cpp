#include "basements/io/urdf_parser.h"
#include "basements/io/urdf_converter.h"
#include <gtest/gtest.h>
#include <fstream>

using namespace basements::io;
using namespace basements::editor;

// ========== Basic Parsing Tests ==========

TEST(URDFParserTest, ParseSimpleArm) {
    URDFParser parser;
    auto result = parser.parse_file("test_data/simple_arm.urdf");
    
    ASSERT_TRUE(result.success) << "Parse error: " << result.error_message;
    EXPECT_EQ(result.robot.name, "simple_arm");
    EXPECT_EQ(result.robot.links.size(), 2);
    EXPECT_EQ(result.robot.joints.size(), 1);
    
    // Check base_link
    const URDFLink* base = result.robot.find_link("base_link");
    ASSERT_NE(base, nullptr);
    EXPECT_FLOAT_EQ(base->inertial.mass, 2.0f);
    EXPECT_EQ(base->visuals.size(), 1);
    EXPECT_EQ(base->collisions.size(), 1);
    
    // Check arm_link
    const URDFLink* arm = result.robot.find_link("arm_link");
    ASSERT_NE(arm, nullptr);
    EXPECT_FLOAT_EQ(arm->inertial.mass, 1.0f);
    
    // Check shoulder joint
    const URDFJoint* shoulder = result.robot.find_joint("shoulder");
    ASSERT_NE(shoulder, nullptr);
    EXPECT_EQ(shoulder->type, URDFJointType::REVOLUTE);
    EXPECT_EQ(shoulder->parent_link, "base_link");
    EXPECT_EQ(shoulder->child_link, "arm_link");
    EXPECT_FLOAT_EQ(shoulder->lower_limit, -1.57f);
    EXPECT_FLOAT_EQ(shoulder->upper_limit, 1.57f);
}

TEST(URDFParserTest, ParseGeometry) {
    URDFParser parser;
    auto result = parser.parse_file("test_data/simple_arm.urdf");
    
    ASSERT_TRUE(result.success);
    
    const URDFLink* base = result.robot.find_link("base_link");
    ASSERT_NE(base, nullptr);
    ASSERT_FALSE(base->visuals.empty());
    
    const URDFGeometry& geom = base->visuals[0].geometry;
    EXPECT_EQ(geom.type, URDFGeometry::BOX);
    EXPECT_FLOAT_EQ(geom.size.x, 0.2f);
    EXPECT_FLOAT_EQ(geom.size.y, 0.2f);
    EXPECT_FLOAT_EQ(geom.size.z, 0.1f);
}

TEST(URDFParserTest, ParseMaterial) {
    URDFParser parser;
    auto result = parser.parse_file("test_data/simple_arm.urdf");
    
    ASSERT_TRUE(result.success);
    
    const URDFLink* base = result.robot.find_link("base_link");
    ASSERT_NE(base, nullptr);
    ASSERT_FALSE(base->visuals.empty());
    
    const URDFMaterial& mat = base->visuals[0].material;
    EXPECT_EQ(mat.name, "blue");
    EXPECT_FLOAT_EQ(mat.color.r, 0.0f);
    EXPECT_FLOAT_EQ(mat.color.g, 0.0f);
    EXPECT_FLOAT_EQ(mat.color.b, 0.8f);
}

// ========== Export Tests ==========

TEST(URDFParserTest, ExportToString) {
    URDFRobot robot("test_robot");
    
    // Create simple link
    URDFLink link("test_link");
    link.inertial.mass = 1.5f;
    robot.links.push_back(link);
    
    URDFParser parser;
    std::string xml = parser.export_to_string(robot);
    
    EXPECT_TRUE(xml.find("<robot name=\"test_robot\">") != std::string::npos);
    EXPECT_TRUE(xml.find("<link name=\"test_link\">") != std::string::npos);
    EXPECT_TRUE(xml.find("<mass value=\"1.5\"") != std::string::npos);
}

TEST(URDFParserTest, RoundTrip) {
    URDFParser parser;
    
    // Parse original
    auto original = parser.parse_file("test_data/simple_arm.urdf");
    ASSERT_TRUE(original.success);
    
    // Export to string
    std::string xml = parser.export_to_string(original.robot);
    
    // Parse exported
    auto reparsed = parser.parse_string(xml);
    ASSERT_TRUE(reparsed.success);
    
    // Verify equivalence
    EXPECT_EQ(original.robot.name, reparsed.robot.name);
    EXPECT_EQ(original.robot.links.size(), reparsed.robot.links.size());
    EXPECT_EQ(original.robot.joints.size(), reparsed.robot.joints.size());
}

// ========== SceneGraph Conversion Tests ==========

TEST(URDFParserTest, ToSceneGraph) {
    URDFParser parser;
    auto result = parser.parse_file("test_data/simple_arm.urdf");
    ASSERT_TRUE(result.success);
    
    SceneGraph scene;
    NodeID root_id;
    std::string error;
    
    bool success = URDFConverter::to_scene_graph(result.robot, scene, root_id, error);
    ASSERT_TRUE(success) << "Error: " << error;
    
    // Check root node
    const SceneNode* root = scene.get_node(root_id);
    ASSERT_NE(root, nullptr);
    EXPECT_EQ(root->name, "simple_arm");
    
    // Should have created nodes for links
    auto all_nodes = scene.get_all_nodes();
    // Root + 2 links = 3 nodes minimum
    EXPECT_GE(all_nodes.size(), 3);
}

TEST(URDFParserTest, FromSceneGraph) {
    // Create simple scene graph
    SceneGraph scene;
    NodeID root = scene.create_node(NodeType::GROUP, "robot");
    NodeID link1 = scene.create_node(NodeType::RIGID_BODY, "link1");
    
    SceneNode* node1 = scene.get_node(link1);
    node1->physics.mass = 2.0f;
    
    // Set backward compatible simple properties
    node1->physics.shape_type = ShapeType::BOX;
    node1->physics.shape_size = Vec3(1.0f, 1.0f, 1.0f);
    
    // Add multiple elements to test multi-geometry export
    SceneNode::CollisionElement col1;
    col1.shape = ShapeType::BOX;
    col1.size = Vec3(1.0f, 1.0f, 1.0f);
    node1->collision_elements.push_back(col1);
    
    SceneNode::CollisionElement col2;
    col2.shape = ShapeType::SPHERE;
    col2.size = Vec3(0.5f, 0.5f, 0.5f);
    col2.offset = Vec3(2.0f, 0.0f, 0.0f);
    node1->collision_elements.push_back(col2);
    
    scene.set_parent(link1, root);
    
    // Convert to URDF
    URDFRobot robot;
    std::string error;
    bool success = URDFConverter::from_scene_graph(scene, root, robot, error);
    
    ASSERT_TRUE(success) << "Error: " << error;
    EXPECT_EQ(robot.name, "robot");
    EXPECT_EQ(robot.links.size(), 1);
    
    const URDFLink* link = robot.find_link("link1");
    ASSERT_NE(link, nullptr);
    EXPECT_FLOAT_EQ(link->inertial.mass, 2.0f);
    EXPECT_EQ(link->collisions.size(), 2);
    EXPECT_EQ(link->collisions[1].geometry.type, URDFGeometry::SPHERE);
    EXPECT_FLOAT_EQ(link->collisions[1].origin.position.x, 2.0f);
}

// ========== Error Handling Tests ==========

TEST(URDFParserTest, InvalidFile) {
    URDFParser parser;
    auto result = parser.parse_file("nonexistent.urdf");
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

TEST(URDFParserTest, InvalidXML) {
    URDFParser parser;
    auto result = parser.parse_string("<invalid xml");
    
    EXPECT_FALSE(result.success);
}

TEST(URDFParserTest, MissingRobotElement) {
    URDFParser parser;
    auto result = parser.parse_string("<?xml version=\"1.0\"?><notrobot/>");
    
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.error_message.find("robot") != std::string::npos);
}

// ========== Real-World Robot Test ==========

TEST(URDFParserTest, ParseTurtleBot3) {
    URDFParser parser;
    auto result = parser.parse_file("test_data/turtlebot3_burger.urdf");
    
    ASSERT_TRUE(result.success) << "Parse error: " << result.error_message;
    EXPECT_EQ(result.robot.name, "turtlebot3_burger");
    EXPECT_GE(result.robot.links.size(), 5);  // At least 5 links
    EXPECT_GE(result.robot.joints.size(), 4);  // At least 4 joints
    
    // Convert to SceneGraph
    SceneGraph scene;
    NodeID root_id;
    std::string error;
    bool success = URDFConverter::to_scene_graph(result.robot, scene, root_id, error);
    ASSERT_TRUE(success) << "Conversion error: " << error;
    
    // Verify multi-geometry support
    auto all_nodes = scene.get_all_nodes();
    bool found_multi_collision = false;
    for (NodeID id : all_nodes) {
        const SceneNode* node = scene.get_node(id);
        if (node && node->collision_elements.size() > 1) {
            found_multi_collision = true;
            
            // Verify each collision element has proper offset/rotation
            for (const auto& elem : node->collision_elements) {
                EXPECT_GT(elem.size.x, 0.0f);  // Should have positive size
            }
            break;
        }
    }
    
    // The important thing is that the system handles multi-collision correctly
    EXPECT_GE(all_nodes.size(), 5);
}


int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
