#include "path_planning_server/static_scene_loader.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <yaml-cpp/yaml.h>

namespace path_planning_server
{

namespace
{

shape_msgs::msg::Mesh makeHexahedron(
  const double lower_min_x,
  const double lower_max_x,
  const double lower_min_y,
  const double lower_max_y,
  const double upper_min_x,
  const double upper_max_x,
  const double upper_min_y,
  const double upper_max_y,
  const double min_z,
  const double max_z)
{
  shape_msgs::msg::Mesh mesh;
  mesh.vertices.resize(8);
  mesh.vertices[0].x = lower_min_x;
  mesh.vertices[0].y = lower_min_y;
  mesh.vertices[0].z = min_z;
  mesh.vertices[1].x = lower_max_x;
  mesh.vertices[1].y = lower_min_y;
  mesh.vertices[1].z = min_z;
  mesh.vertices[2].x = lower_max_x;
  mesh.vertices[2].y = lower_max_y;
  mesh.vertices[2].z = min_z;
  mesh.vertices[3].x = lower_min_x;
  mesh.vertices[3].y = lower_max_y;
  mesh.vertices[3].z = min_z;
  mesh.vertices[4].x = upper_min_x;
  mesh.vertices[4].y = upper_min_y;
  mesh.vertices[4].z = max_z;
  mesh.vertices[5].x = upper_max_x;
  mesh.vertices[5].y = upper_min_y;
  mesh.vertices[5].z = max_z;
  mesh.vertices[6].x = upper_max_x;
  mesh.vertices[6].y = upper_max_y;
  mesh.vertices[6].z = max_z;
  mesh.vertices[7].x = upper_min_x;
  mesh.vertices[7].y = upper_max_y;
  mesh.vertices[7].z = max_z;

  constexpr std::array<std::array<std::uint32_t, 3>, 12> triangles = {{
    {{0, 2, 1}}, {{0, 3, 2}},
    {{4, 5, 6}}, {{4, 6, 7}},
    {{0, 1, 5}}, {{0, 5, 4}},
    {{1, 2, 6}}, {{1, 6, 5}},
    {{2, 3, 7}}, {{2, 7, 6}},
    {{3, 0, 4}}, {{3, 4, 7}}
  }};
  mesh.triangles.reserve(triangles.size());
  for (const auto & indices : triangles) {
    shape_msgs::msg::MeshTriangle triangle;
    triangle.vertex_indices = indices;
    mesh.triangles.push_back(triangle);
  }
  return mesh;
}

moveit_msgs::msg::CollisionObject makeOpenTrapezoidBox(const YAML::Node & config)
{
  const double bottom_length = config["bottom_length"].as<double>();
  const double bottom_width = config["bottom_width"].as<double>();
  const double top_length = config["top_length"].as<double>();
  const double top_width = config["top_width"].as<double>();
  const double height = config["height"].as<double>();
  const double thickness = config["thickness"].as<double>();
  if (
    !std::isfinite(bottom_length) || !std::isfinite(bottom_width) ||
    !std::isfinite(top_length) || !std::isfinite(top_width) ||
    !std::isfinite(height) || !std::isfinite(thickness) ||
    bottom_length <= 0.0 || bottom_width <= 0.0 ||
    top_length <= 0.0 || top_width <= 0.0 ||
    height <= 0.0 || thickness <= 0.0)
  {
    throw std::runtime_error("Open trapezoid box dimensions must be finite and positive");
  }
  if (config["long_axis"].as<std::string>() != "y") {
    throw std::runtime_error("Open trapezoid box long_axis must be 'y'");
  }

  const double length_slope = (top_length - bottom_length) / (2.0 * height);
  const double width_slope = (top_width - bottom_width) / (2.0 * height);
  const double end_wall_inset = thickness * std::sqrt(1.0 + length_slope * length_slope);
  const double side_wall_inset = thickness * std::sqrt(1.0 + width_slope * width_slope);
  if (
    2.0 * end_wall_inset >= std::min(bottom_length, top_length) ||
    2.0 * side_wall_inset >= std::min(bottom_width, top_width) ||
    thickness >= height)
  {
    throw std::runtime_error("Open trapezoid box thickness is too large");
  }

  const double lower_half_length = bottom_length / 2.0;
  const double upper_half_length = top_length / 2.0;
  const double lower_half_width = bottom_width / 2.0;
  const double upper_half_width = top_width / 2.0;

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = config["frame_id"].as<std::string>();
  collision_object.id = config["id"].as<std::string>();

  // Bottom plate.
  collision_object.meshes.push_back(makeHexahedron(
    -lower_half_width, lower_half_width,
    -lower_half_length, lower_half_length,
    -lower_half_width, lower_half_width,
    -lower_half_length, lower_half_length,
    0.0, thickness));

  // Walls at the two constant-width sides. Their end points follow the
  // trapezoid's lower and upper lengths.
  collision_object.meshes.push_back(makeHexahedron(
    lower_half_width - side_wall_inset, lower_half_width,
    -lower_half_length, lower_half_length,
    upper_half_width - side_wall_inset, upper_half_width,
    -upper_half_length, upper_half_length,
    0.0, height));
  collision_object.meshes.push_back(makeHexahedron(
    -lower_half_width, -lower_half_width + side_wall_inset,
    -lower_half_length, lower_half_length,
    -upper_half_width, -upper_half_width + side_wall_inset,
    -upper_half_length, upper_half_length,
    0.0, height));

  // Sloped walls at the two ends of the long axis.
  collision_object.meshes.push_back(makeHexahedron(
    -lower_half_width, lower_half_width,
    lower_half_length - end_wall_inset, lower_half_length,
    -upper_half_width, upper_half_width,
    upper_half_length - end_wall_inset, upper_half_length,
    0.0, height));
  collision_object.meshes.push_back(makeHexahedron(
    -lower_half_width, lower_half_width,
    -lower_half_length, -lower_half_length + end_wall_inset,
    -upper_half_width, upper_half_width,
    -upper_half_length, -upper_half_length + end_wall_inset,
    0.0, height));

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = config["center_x"].as<double>();
  pose.position.y = config["center_y"].as<double>();
  pose.position.z = config["bottom_z"].as<double>();
  collision_object.mesh_poses.assign(collision_object.meshes.size(), pose);
  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return collision_object;
}

moveit_msgs::msg::CollisionObject makeShelf(const YAML::Node & config)
{
  const double length = config["length"].as<double>();
  const double width = config["width"].as<double>();
  const double height = config["height"].as<double>();
  const double board_length = config["board_length"].as<double>();
  const double board_width = config["board_width"].as<double>();
  const double board_thickness = config["board_thickness"].as<double>();
  const double post_diameter = config["post_diameter"].as<double>();
  const double center_x = config["center_x"].as<double>();
  const double center_y = config["center_y"].as<double>();
  const double bottom_z = config["bottom_z"].as<double>();
  const auto board_center_heights =
    config["board_center_heights"].as<std::vector<double>>();

  const std::array<double, 7> dimensions = {
    length, width, height, board_length, board_width, board_thickness, post_diameter
  };
  if (std::any_of(
      dimensions.begin(), dimensions.end(),
      [](const double value) {return !std::isfinite(value) || value <= 0.0;}))
  {
    throw std::runtime_error("Shelf dimensions must be finite and positive");
  }
  if (
    !std::isfinite(center_x) || !std::isfinite(center_y) ||
    !std::isfinite(bottom_z))
  {
    throw std::runtime_error("Shelf position must be finite");
  }
  if (config["long_axis"].as<std::string>() != "y") {
    throw std::runtime_error("Shelf long_axis must be 'y'");
  }
  if (
    board_length > length || board_width > width ||
    board_thickness > height || post_diameter > length || post_diameter > width)
  {
    throw std::runtime_error("Shelf components exceed the requested outer dimensions");
  }
  for (const double center_height : board_center_heights) {
    if (
      !std::isfinite(center_height) ||
      center_height - board_thickness / 2.0 < 0.0 ||
      center_height + board_thickness / 2.0 > height)
    {
      throw std::runtime_error("Shelf board lies outside the requested height");
    }
  }

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = config["frame_id"].as<std::string>();
  collision_object.id = config["id"].as<std::string>();

  for (const double center_height : board_center_heights) {
    shape_msgs::msg::SolidPrimitive board;
    board.type = shape_msgs::msg::SolidPrimitive::BOX;
    board.dimensions = {board_width, board_length, board_thickness};

    geometry_msgs::msg::Pose board_pose;
    board_pose.orientation.w = 1.0;
    board_pose.position.x = center_x;
    board_pose.position.y = center_y;
    board_pose.position.z = bottom_z + center_height;

    collision_object.primitives.push_back(board);
    collision_object.primitive_poses.push_back(board_pose);
  }

  const double post_radius = post_diameter / 2.0;
  const double post_offset_x = width / 2.0 - post_radius;
  const double post_offset_y = length / 2.0 - post_radius;
  for (const double x_sign : {-1.0, 1.0}) {
    for (const double y_sign : {-1.0, 1.0}) {
      shape_msgs::msg::SolidPrimitive post;
      post.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      post.dimensions = {height, post_radius};

      geometry_msgs::msg::Pose post_pose;
      post_pose.orientation.w = 1.0;
      post_pose.position.x = center_x + x_sign * post_offset_x;
      post_pose.position.y = center_y + y_sign * post_offset_y;
      post_pose.position.z = bottom_z + height / 2.0;

      collision_object.primitives.push_back(post);
      collision_object.primitive_poses.push_back(post_pose);
    }
  }

  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return collision_object;
}

}  // namespace

StaticSceneLoader::StaticSceneLoader(
  const std::string & scene_index_file,
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene)
: planning_scene_(std::move(planning_scene))
{
  const auto index_path = resolveIndexPath(scene_index_file);
  const auto index = YAML::LoadFile(index_path.string());
  const auto scenes = index["scenes"];
  if (!scenes || !scenes.IsMap()) {
    throw std::runtime_error("scene_index.yaml must contain a scenes map");
  }

  for (const auto & entry : scenes) {
    const auto scene_id = entry.first.as<std::uint32_t>();
    const auto file = entry.second["file"].as<std::string>();
    scene_files_.emplace(scene_id, index_path.parent_path() / file);
  }
}

bool StaticSceneLoader::hasScene(const std::uint32_t scene_id) const
{
  return scene_files_.find(scene_id) != scene_files_.end();
}

bool StaticSceneLoader::loadScene(const std::uint32_t scene_id, std::string & message)
{
  const auto scene_it = scene_files_.find(scene_id);
  if (scene_it == scene_files_.end()) {
    message = "Scene ID not found";
    return false;
  }
  if (loaded_scene_id_ && *loaded_scene_id_ == scene_id) {
    return true;
  }

  try {
    const auto scene = YAML::LoadFile(scene_it->second.string());
    std::vector<moveit_msgs::msg::CollisionObject> next_objects;

    const auto make_box = [](
      const YAML::Node & config,
      const double size_z,
      const double center_z)
      {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = config["frame_id"].as<std::string>();
        collision_object.id = config["id"].as<std::string>();

        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {
          config["size_x"].as<double>(),
          config["size_y"].as<double>(),
          size_z
        };

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = config["center_x"].as<double>();
        pose.position.y = config["center_y"].as<double>();
        pose.position.z = center_z;

        collision_object.primitives.push_back(box);
        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
        return collision_object;
      };

    const auto table = scene["table"];
    const bool table_enabled = table && (!table["enabled"] || table["enabled"].as<bool>());
    if (table_enabled) {
      if (table["type"].as<std::string>() != "box") {
        message = "Only box tables are supported";
        return false;
      }
      const double thickness = table["thickness"].as<double>();
      next_objects.push_back(make_box(
        table, thickness,
        table["top_z"].as<double>() - thickness / 2.0));
    }

    const auto open_trapezoid_box = scene["open_trapezoid_box"];
    const bool open_trapezoid_box_enabled =
      open_trapezoid_box &&
      (!open_trapezoid_box["enabled"] || open_trapezoid_box["enabled"].as<bool>());
    if (open_trapezoid_box_enabled) {
      if (open_trapezoid_box["type"].as<std::string>() != "open_trapezoid_box") {
        message = "Unsupported open trapezoid box type";
        return false;
      }
      next_objects.push_back(makeOpenTrapezoidBox(open_trapezoid_box));
    }

    const auto shelf = scene["shelf"];
    const bool shelf_enabled = shelf && (!shelf["enabled"] || shelf["enabled"].as<bool>());
    if (shelf_enabled) {
      if (shelf["type"].as<std::string>() != "shelf") {
        message = "Unsupported shelf type";
        return false;
      }
      next_objects.push_back(makeShelf(shelf));
    }

    const auto obstacle = scene["obstacle"];
    const bool obstacle_enabled =
      obstacle && (!obstacle["enabled"] || obstacle["enabled"].as<bool>());
    if (obstacle_enabled) {
      if (obstacle["type"].as<std::string>() != "box") {
        message = "Only box obstacles are supported";
        return false;
      }
      next_objects.push_back(make_box(
        obstacle,
        obstacle["size_z"].as<double>(),
        obstacle["center_z"].as<double>()));
    }

    const auto camera_obstacle = scene["camera_obstacle"];
    const bool camera_obstacle_enabled =
      camera_obstacle &&
      (!camera_obstacle["enabled"] ||
      camera_obstacle["enabled"].as<bool>());

    if (camera_obstacle_enabled) {
      if (camera_obstacle["type"].as<std::string>() != "box") {
        message = "Only box camera obstacles are supported";
        return false;
      }

      next_objects.push_back(make_box(
        camera_obstacle,
        camera_obstacle["size_z"].as<double>(),
        camera_obstacle["center_z"].as<double>()));
    }

    std::vector<std::string> next_object_ids;
    next_object_ids.reserve(next_objects.size());
    for (const auto & object : next_objects) {
      next_object_ids.push_back(object.id);
    }

    std::vector<moveit_msgs::msg::CollisionObject> updates;
    for (const auto & loaded_id : loaded_object_ids_) {
      if (std::find(next_object_ids.begin(), next_object_ids.end(), loaded_id) ==
        next_object_ids.end())
      {
        moveit_msgs::msg::CollisionObject removal;
        removal.id = loaded_id;
        removal.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        updates.push_back(removal);
      }
    }
    updates.insert(updates.end(), next_objects.begin(), next_objects.end());

    if (!updates.empty() && !planning_scene_->applyCollisionObjects(updates)) {
      message = "Failed to apply static scene";
      return false;
    }

    loaded_object_ids_ = std::move(next_object_ids);
  } catch (const YAML::Exception & error) {
    message = std::string("Failed to load static scene: ") + error.what();
    return false;
  } catch (const std::exception & error) {
    message = std::string("Failed to construct static scene: ") + error.what();
    return false;
  }

  loaded_scene_id_ = scene_id;
  return true;
}

std::filesystem::path StaticSceneLoader::resolveIndexPath(
  const std::string & scene_index_file) const
{
  std::filesystem::path path(scene_index_file);
  if (path.is_absolute()) {
    return path;
  }
  return std::filesystem::path(
    ament_index_cpp::get_package_share_directory("path_planning_server")) / path;
}

}  // namespace path_planning_server
