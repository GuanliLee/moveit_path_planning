#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

namespace path_planning_server
{

class StaticSceneLoader
{
public:
  StaticSceneLoader(
    const std::string & scene_index_file,
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene);

  bool hasScene(std::uint32_t scene_id) const;
  bool loadScene(std::uint32_t scene_id, std::string & message);

private:
  std::filesystem::path resolveIndexPath(const std::string & scene_index_file) const;

  std::map<std::uint32_t, std::filesystem::path> scene_files_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
  std::optional<std::uint32_t> loaded_scene_id_;
  std::vector<std::string> loaded_object_ids_;
};

}  // namespace path_planning_server
