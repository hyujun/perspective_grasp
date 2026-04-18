#pragma once

#include <string>
#include <unordered_map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace perspective_grasp {

/// Loads and caches CAD model point clouds (.pcd) keyed by class name
class CadModelManager {
 public:
  /// Load all .pcd files from the given directory
  /// Filename (minus extension) becomes the class name key
  void loadModels(const std::string& model_dir);

  /// Get a cached model by class name, or nullptr if not found
  pcl::PointCloud<pcl::PointXYZ>::Ptr getModel(
      const std::string& class_name) const;

  /// Check if any models are loaded
  bool hasModels() const { return !models_.empty(); }

  /// Get number of loaded models
  size_t modelCount() const { return models_.size(); }

 private:
  std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>
      models_;
};

}  // namespace perspective_grasp
