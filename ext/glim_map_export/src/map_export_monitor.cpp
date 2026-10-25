// Copyright 2026 Chris Tomaszewski
// Licensed under the Apache License, Version 2.0.
//
// GLIM extension module: keep the latest optimized submaps and, when GLIM saves (at_exit, after
// the final optimization), write them merged in the world frame as <session>/map.ply — a compact
// point cloud any viewer opens, where GLIM's own dump needs its desktop offline_viewer.
//
// Settings (config_ros.json, section "glim_service", written by the supervisor):
//   export_ply      bool    write the PLY (default true)
//   export_voxel_m  double  voxel-grid downsampling, first point per cell; 0 = every point
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include <gtsam_points/util/vector3i_hash.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/util/config.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/logging.hpp>

namespace glim_service {

class MapExportMonitor : public glim::ExtensionModule {
public:
  MapExportMonitor() : logger(glim::create_module_logger("map_export")) {
    const glim::Config config(glim::GlobalConfig::get_config_path("config_ros"));
    enabled = config.param<bool>("glim_service", "export_ply", true);
    voxel = config.param<double>("glim_service", "export_voxel_m", 0.1);
    // on_update_submaps carries the complete, re-optimized submap list every time.
    glim::GlobalMappingCallbacks::on_update_submaps.add([this](const std::vector<glim::SubMap::Ptr>& submaps) {
      std::lock_guard<std::mutex> lock(mutex);
      latest = submaps;
    });
  }

  void at_exit(const std::string& dump_path) override {
    if (!enabled) {
      return;
    }
    std::vector<glim::SubMap::Ptr> submaps;
    {
      std::lock_guard<std::mutex> lock(mutex);
      submaps = latest;
    }
    if (submaps.empty()) {
      logger->info("no submaps (odometry-only pipeline, or no data): no map.ply");
      return;
    }

    bool intensities = true;
    size_t total = 0;
    for (const auto& submap : submaps) {
      if (!submap || !submap->frame) {
        continue;
      }
      total += submap->frame->size();
      intensities = intensities && submap->frame->has_intensities();
    }

    std::vector<Eigen::Vector3f> points;
    std::vector<float> values;
    points.reserve(total);
    std::unordered_set<Eigen::Vector3i, gtsam_points::Vector3iHash> occupied;
    const double inv = voxel > 0.0 ? 1.0 / voxel : 0.0;
    for (const auto& submap : submaps) {
      if (!submap || !submap->frame) {
        continue;
      }
      const auto& frame = *submap->frame;
      for (size_t i = 0; i < frame.size(); i++) {
        const Eigen::Vector3d p = submap->T_world_origin * frame.points[i].head<3>();
        if (inv > 0.0) {
          const Eigen::Vector3i cell = (p * inv).array().floor().cast<int>();
          if (!occupied.insert(cell).second) {
            continue;
          }
        }
        points.push_back(p.cast<float>());
        if (intensities) {
          values.push_back(static_cast<float>(frame.intensities[i]));
        }
      }
    }

    const auto target = std::filesystem::path(dump_path).parent_path() / "map.ply";
    const auto partial = std::filesystem::path(target.string() + ".partial");
    {
      std::ofstream ofs(partial, std::ios::binary);
      ofs << "ply\nformat binary_little_endian 1.0\n"
          << "comment glim-service map export, world frame, metres\n"
          << "element vertex " << points.size() << "\n"
          << "property float x\nproperty float y\nproperty float z\n";
      if (intensities) {
        ofs << "property float intensity\n";
      }
      ofs << "end_header\n";
      for (size_t i = 0; i < points.size(); i++) {
        ofs.write(reinterpret_cast<const char*>(points[i].data()), sizeof(float) * 3);
        if (intensities) {
          ofs.write(reinterpret_cast<const char*>(&values[i]), sizeof(float));
        }
      }
      if (!ofs) {
        logger->error("failed to write {}", partial.string());
        return;
      }
    }
    std::filesystem::rename(partial, target);  // readers never see a half-written map
    logger->info("exported {} of {} points from {} submaps to {} (voxel {} m)", points.size(), total,
                 submaps.size(), target.string(), voxel);
  }

private:
  std::shared_ptr<spdlog::logger> logger;
  bool enabled = true;
  double voxel = 0.1;
  std::mutex mutex;
  std::vector<glim::SubMap::Ptr> latest;
};

}  // namespace glim_service

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim_service::MapExportMonitor();
}
