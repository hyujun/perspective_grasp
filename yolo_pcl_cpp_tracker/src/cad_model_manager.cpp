#include "yolo_pcl_cpp_tracker/cad_model_manager.hpp"

#include <filesystem>

#include <pcl/io/pcd_io.h>

namespace perspective_grasp {

void CadModelManager::loadModels(const std::string& model_dir) {
  models_.clear();

  if (!std::filesystem::exists(model_dir)) {
    return;
  }

  for (const auto& entry : std::filesystem::directory_iterator(model_dir)) {
    if (entry.path().extension() == ".pcd") {
      auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(entry.path().string(),
                                                *cloud) == 0) {
        std::string class_name = entry.path().stem().string();
        models_[class_name] = cloud;
      }
    }
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CadModelManager::getModel(
    const std::string& class_name) const {
  auto it = models_.find(class_name);
  if (it != models_.end()) {
    return it->second;
  }
  return nullptr;
}

}  // namespace perspective_grasp
