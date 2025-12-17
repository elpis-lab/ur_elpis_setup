#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <boost/variant/get.hpp>
#include <stdexcept>
#include <geometry_msgs/msg/pose.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <string>
#include <vector>

class SetupEnvironmentNode : public rclcpp::Node
{
public:
  SetupEnvironmentNode() : Node("setup_environment_node"), planning_scene_interface_("")
  {
    std::string config_file = this->declare_parameter<std::string>(
      "config_file", 
      ament_index_cpp::get_package_share_directory("setup_environment") + "/config/environment_config.yaml"
    );
    
    RCLCPP_INFO(get_logger(), "Loading environment config from: %s", config_file.c_str());
    
    // Load and parse config
    YAML::Node config = YAML::LoadFile(config_file);
    
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    
    // Setup table
    if (config["table"])
    {
      setup_object(config["table"], collision_objects);
    }
    
    // Setup ceiling
    if (config["ceiling"])
    {
      setup_object(config["ceiling"], collision_objects);
    }
    
    // Apply all collision objects
    if (!collision_objects.empty())
    {
      bool success = planning_scene_interface_.applyCollisionObjects(collision_objects);
      if (success)
      {
        RCLCPP_INFO(get_logger(), "Successfully added %zu environment objects to planning scene", 
                    collision_objects.size());
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Failed to add environment objects to planning scene");
      }
    }
    else
    {
      RCLCPP_WARN(get_logger(), "No environment objects to add");
    }
  }

private:
  void setup_object(const YAML::Node& obj_config, std::vector<moveit_msgs::msg::CollisionObject>& collision_objects)
  {
    moveit_msgs::msg::CollisionObject collision_obj;
    collision_obj.header.frame_id = obj_config["frame_id"].as<std::string>("world");
    collision_obj.header.stamp = now();
    collision_obj.id = obj_config["id"].as<std::string>();
    collision_obj.operation = collision_obj.ADD;
    
    std::string type = obj_config["type"].as<std::string>();
    
    if (type == "mesh")
    {
      setup_mesh(obj_config, collision_obj);
    }
    else if (type == "primitive")
    {
      setup_primitive(obj_config, collision_obj);
    }
    else
    {
      throw std::runtime_error("Unknown object type: " + type + " (must be 'mesh' or 'primitive')");
    }
    
    collision_objects.push_back(collision_obj);
  }
  
  void setup_mesh(const YAML::Node& obj_config, moveit_msgs::msg::CollisionObject& collision_obj)
  {
    if (!obj_config["mesh"])
    {
      throw std::runtime_error("Mesh type specified but 'mesh' configuration not found for object: " + 
                               obj_config["id"].as<std::string>());
    }
    
    std::string mesh_path = obj_config["mesh"]["resource_path"].as<std::string>();
    shapes::Mesh* m = shapes::createMeshFromResource(mesh_path);
    
    if (m == nullptr)
    {
      throw std::runtime_error("Failed to load mesh from: " + mesh_path);
    }
    
    // Convert mesh to message
    shape_msgs::msg::Mesh mesh_msg;
    shapes::ShapeMsg mesh_shape_msg;
    shapes::constructMsgFromShape(m, mesh_shape_msg);
    mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_shape_msg);
    
    // Clear primitives when using meshes
    collision_obj.primitives.clear();
    collision_obj.primitive_poses.clear();
    
    // Add mesh
    collision_obj.meshes.resize(1);
    collision_obj.meshes[0] = mesh_msg;
    collision_obj.mesh_poses.resize(1);
    
    // Set mesh pose from config
    auto pose_node = obj_config["mesh"]["pose"];
    collision_obj.mesh_poses[0].position.x = pose_node["position"]["x"].as<double>();
    collision_obj.mesh_poses[0].position.y = pose_node["position"]["y"].as<double>();
    collision_obj.mesh_poses[0].position.z = pose_node["position"]["z"].as<double>();
    collision_obj.mesh_poses[0].orientation.x = pose_node["orientation"]["x"].as<double>(0.0);
    collision_obj.mesh_poses[0].orientation.y = pose_node["orientation"]["y"].as<double>(0.0);
    collision_obj.mesh_poses[0].orientation.z = pose_node["orientation"]["z"].as<double>(0.0);
    collision_obj.mesh_poses[0].orientation.w = pose_node["orientation"]["w"].as<double>(1.0);
    
    delete m;
    RCLCPP_INFO(get_logger(), "Successfully loaded mesh for %s with %zu triangles", 
                collision_obj.id.c_str(), mesh_msg.triangles.size());
  }
  
  void setup_primitive(const YAML::Node& obj_config, moveit_msgs::msg::CollisionObject& collision_obj)
  {
    if (!obj_config["primitive"])
    {
      throw std::runtime_error("Primitive type specified but 'primitive' configuration not found for object: " + 
                               obj_config["id"].as<std::string>());
    }
    
    std::string shape = obj_config["primitive"]["shape"].as<std::string>();
    shape_msgs::msg::SolidPrimitive primitive;
    
    auto dims = obj_config["primitive"]["dimensions"];
    
    if (shape == "box")
    {
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = dims["x"].as<double>();
      primitive.dimensions[1] = dims["y"].as<double>();
      primitive.dimensions[2] = dims["z"].as<double>();
    }
    else if (shape == "cylinder")
    {
      primitive.type = primitive.CYLINDER;
      primitive.dimensions.resize(2);
      primitive.dimensions[0] = dims["x"].as<double>();  // radius
      primitive.dimensions[1] = dims["y"].as<double>();  // height
    }
    else
    {
      throw std::runtime_error("Unknown primitive shape: " + shape + " (must be 'box' or 'cylinder')");
    }
    
    collision_obj.primitives.push_back(primitive);
    
    // Set pose from config (use top-level pose or primitive.pose)
    geometry_msgs::msg::Pose pose;
    auto pose_node = obj_config["primitive"]["pose"];
    if (!pose_node)
    {
      pose_node = obj_config["pose"];  // Fallback to top-level pose
    }
    
    pose.position.x = pose_node["position"]["x"].as<double>();
    pose.position.y = pose_node["position"]["y"].as<double>();
    pose.position.z = pose_node["position"]["z"].as<double>();
    pose.orientation.x = pose_node["orientation"]["x"].as<double>(0.0);
    pose.orientation.y = pose_node["orientation"]["y"].as<double>(0.0);
    pose.orientation.z = pose_node["orientation"]["z"].as<double>(0.0);
    pose.orientation.w = pose_node["orientation"]["w"].as<double>(1.0);
    
    collision_obj.primitive_poses.push_back(pose);
    
    RCLCPP_INFO(get_logger(), "Added %s primitive (%s) for %s at (%.3f, %.3f, %.3f)", 
                shape.c_str(), collision_obj.id.c_str(), collision_obj.id.c_str(),
                pose.position.x, pose.position.y, pose.position.z);
  }
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<SetupEnvironmentNode>();
  
  // Node runs once to setup environment, then exits
  // The collision objects remain in the planning scene
  RCLCPP_INFO(node->get_logger(), "Environment setup complete. Node shutting down.");
  
  rclcpp::shutdown();
  return 0;
}

