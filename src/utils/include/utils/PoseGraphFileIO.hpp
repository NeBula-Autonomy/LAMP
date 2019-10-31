#pragma once

#include <fstream>

#include <minizip/unzip.h>
#include <minizip/zip.h>

// This new header allows us to read examples easily from .graph files
#include <gtsam/slam/dataset.h>

#include "utils/CommonStructs.h"



namespace {
std::string absPath(const std::string& relPath) {
  return boost::filesystem::canonical(boost::filesystem::path(relPath))
      .string();
}

bool writeFileToZip(zipFile& zip, const std::string& filename) {
  // this code is inspired by
  // http://www.vilipetek.com/2013/11/22/zippingunzipping-files-in-c/
  static const unsigned int BUFSIZE = 2048;

  zip_fileinfo zi = {0};
  tm_zip& tmZip = zi.tmz_date;
  time_t rawtime;
  time(&rawtime);
  auto timeinfo = localtime(&rawtime);
  tmZip.tm_sec = timeinfo->tm_sec;
  tmZip.tm_min = timeinfo->tm_min;
  tmZip.tm_hour = timeinfo->tm_hour;
  tmZip.tm_mday = timeinfo->tm_mday;
  tmZip.tm_mon = timeinfo->tm_mon;
  tmZip.tm_year = timeinfo->tm_year;
  int err = zipOpenNewFileInZip(zip,
                                filename.c_str(),
                                &zi,
                                nullptr,
                                0,
                                nullptr,
                                0,
                                nullptr,
                                Z_DEFLATED,
                                Z_DEFAULT_COMPRESSION);

  if (err != ZIP_OK) {
    ROS_ERROR_STREAM("Failed to add entry \"" << filename << "\" to zip file.");
    return false;
  }
  char buf[BUFSIZE];
  unsigned long nRead = 0;

  std::ifstream is(filename);
  if (is.bad()) {
    ROS_ERROR_STREAM("Could not read file \"" << filename
                                              << "\" to be added to zip file.");
    return false;
  }
  while (err == ZIP_OK && is.good()) {
    is.read(buf, BUFSIZE);
    unsigned int nRead = (unsigned int)is.gcount();
    if (nRead)
      err = zipWriteInFileInZip(zip, buf, nRead);
    else
      break;
  }
  is.close();
  if (err != ZIP_OK) {
    ROS_ERROR_STREAM("Failed to write file \"" << filename
                                               << "\" to zip file.");
    return false;
  }
  return true;
}
} // namespace

template <typename PGOSolver>
bool PoseGraph::Save(const std::string& zipFilename, PGOSolver& solver) const {
  const std::string path = "pose_graph";
  const boost::filesystem::path directory(path);
  boost::filesystem::create_directory(directory);

  solver.getFactorsUnsafe().print("In save, solvernfg is: ");
  nfg_.print("In save, from nfg is: ");

  writeG2o(solver.getFactorsUnsafe(),
           solver.calculateEstimate(),
           path + "/graph.g2o");
  ROS_INFO("Saved factor graph as a g2o file.");

  // keys.csv stores factor key, point cloud filename, and time stamp
  std::ofstream keys_file(path + "/keys.csv");
  if (keys_file.bad()) {
    ROS_ERROR("Failed to write keys file.");
    return false;
  }

  auto zipFile = zipOpen64(zipFilename.c_str(), 0);
  writeFileToZip(zipFile, path + "/graph.g2o");
  int i = 0;
  for (const auto& entry : keyed_scans) {
    keys_file << entry.first << ",";
    // save point cloud as binary PCD file
    const std::string pcd_filename = path + "/pc_" + std::to_string(i) + ".pcd";
    pcl::io::savePCDFile(pcd_filename, *entry.second, true);
    writeFileToZip(zipFile, pcd_filename);

    ROS_INFO("Saved point cloud %i/%lu.", i + 1, keyed_scans.size());
    keys_file << pcd_filename << ",";
    if (!values_.exists(entry.first)) {
      ROS_WARN("Key,  %lu, does not exist in Save", entry.first);
      return false;
    }
    keys_file << keyed_stamps.at(entry.first).toNSec() << "\n";
    ++i;
  }
  keys_file.close();
  writeFileToZip(zipFile, path + "/keys.csv");

  // TODO: Save edges (all types of edges)
  // save odometry edges
  //   std::ofstream odometry_edges_file(path + "/odometry_edges.csv");
  //   if (odometry_edges_file.bad()) {
  //     ROS_ERROR("Failed to write odometry_edges file.");
  //     return false;
  //   }
  //   for (const auto &entry : odometry_edges_) {
  //     odometry_edges_file << entry.first << ',' << entry.second << '\n';
  //   }
  //   odometry_edges_file.close();
  //   writeFileToZip(zipFile, path + "/odometry_edges.csv");

  //   // save loop edges
  //   std::ofstream loop_edges_file(path + "/loop_edges.csv");
  //   if (loop_edges_file.bad()) {
  //     ROS_ERROR("Failed to write loop_edges file.");
  //     return false;
  //   }
  //   for (const auto &entry : loop_edges_) {
  //     loop_edges_file << entry.first << ',' << entry.second << '\n';
  //   }
  //   loop_edges_file.close();
  //   writeFileToZip(zipFile, path + "/loop_edges.csv");

  zipClose(zipFile, 0);
  boost::filesystem::remove_all(directory);
  ROS_INFO_STREAM("Successfully saved pose graph to " << absPath(zipFilename)
                                                      << ".");
}

template <typename PGOSolver>
bool PoseGraph::Load(const std::string& zipFilename, PGOSolver& solver) {
  const std::string absFilename = absPath(zipFilename);
  auto zipFile = unzOpen64(zipFilename.c_str());
  // TODO: Storing current key before loading graph to set key to this after
  // stored_key = key;
  if (!zipFile) {
    ROS_ERROR_STREAM("Failed to open zip file " << absFilename);
    return false;
  }

  unz_global_info64 oGlobalInfo;
  int err = unzGetGlobalInfo64(zipFile, &oGlobalInfo);
  std::vector<std::string> files; // files to be extracted

  std::string graphFilename{""}, keysFilename{""}, odometryEdgesFilename{""},
      loopEdgesFilename{""};

  for (unsigned long i = 0; i < oGlobalInfo.number_entry && err == UNZ_OK;
       ++i) {
    char filename[256];
    unz_file_info64 oFileInfo;
    err = unzGetCurrentFileInfo64(zipFile,
                                  &oFileInfo,
                                  filename,
                                  sizeof(filename),
                                  nullptr,
                                  0,
                                  nullptr,
                                  0);
    if (err == UNZ_OK) {
      char nLast = filename[oFileInfo.size_filename - 1];
      // this entry is a file, extract it later
      files.emplace_back(filename);
      if (files.back().find("graph.g2o") != std::string::npos) {
        graphFilename = files.back();
      } else if (files.back().find("keys.csv") != std::string::npos) {
        keysFilename = files.back();
      } else if (files.back().find("odometry_edges.csv") != std::string::npos) {
        odometryEdgesFilename = files.back();
      } else if (files.back().find("loop_edges.csv") != std::string::npos) {
        loopEdgesFilename = files.back();
      }
      err = unzGoToNextFile(zipFile);
    }
  }

  if (graphFilename.empty()) {
    ROS_ERROR_STREAM("Could not find pose graph g2o-file in " << absFilename);
    return false;
  }
  if (keysFilename.empty()) {
    ROS_ERROR_STREAM("Could not find keys.csv in " << absFilename);
    return false;
  }

  // extract files
  int i = 1;
  std::vector<boost::filesystem::path> folders;
  for (const auto& filename : files) {
    if (unzLocateFile(zipFile, filename.c_str(), 0) != UNZ_OK) {
      ROS_ERROR_STREAM("Could not locate file " << filename << " from "
                                                << absFilename);
      return false;
    }
    if (unzOpenCurrentFile(zipFile) != UNZ_OK) {
      ROS_ERROR_STREAM("Could not open file " << filename << " from "
                                              << absFilename);
      return false;
    }
    unz_file_info64 oFileInfo;
    if (unzGetCurrentFileInfo64(zipFile, &oFileInfo, 0, 0, 0, 0, 0, 0) !=
        UNZ_OK) {
      ROS_ERROR_STREAM("Could not determine file size of entry "
                       << filename << " in " << absFilename);
      return false;
    }

    boost::filesystem::path dir(filename);
    dir = dir.parent_path();
    if (boost::filesystem::create_directory(dir))
      folders.emplace_back(dir);

    auto size = (unsigned int)oFileInfo.uncompressed_size;
    char* buf = new char[size];
    size = unzReadCurrentFile(zipFile, buf, size);
    std::ofstream os(filename);
    if (os.bad()) {
      ROS_ERROR_STREAM("Could not create file " << filename
                                                << " for extraction.");
      return false;
    }
    if (size > 0) {
      os.write(buf, size);
      os.flush();
    } else {
      ROS_WARN_STREAM("Entry " << filename << " from " << absFilename
                               << " is empty.");
    }
    os.close();
    delete[] buf;
    ROS_INFO_STREAM("Extracted file " << i << "/" << (int)files.size() << " -- "
                                      << filename);
    ++i;
  }
  unzClose(zipFile);

  // restore pose graph from g2o file
  const gtsam::GraphAndValues gv = gtsam::load3D(graphFilename);
  nfg_ = *gv.first;
  nfg_.print("File output facrtors are: ");
  values_ = *gv.second;
  ROS_INFO_STREAM("1");

  // TODO solver needs to be reset
  // solver.reset(new RobustPGO::RobustSolver(rpgo_params_));
  solver.print();
  ROS_INFO_STREAM("2");

  // TODO: Should store initial_noise to use in load
  const Diagonal::shared_ptr covariance(Diagonal::Sigmas(initial_noise));
  const gtsam::Symbol key0 = gtsam::Symbol(*nfg_.keys().begin());
  // TODO assign it
  // first_loaded_key = key0;
  ROS_INFO_STREAM("3");
  if (!values_.exists(key0)) {
    ROS_WARN("Key0, %s, does not exist in Load",
             gtsam::DefaultKeyFormatter(key0));
    return false;
  }
  // update with prior
  solver.template loadGraph<gtsam::Pose3>(
      nfg_,
      values_,
      gtsam::PriorFactor<gtsam::Pose3>(key0, GetPose(key0), covariance));

  ROS_INFO_STREAM("Updated graph from " << graphFilename);

  // info_file stores factor key, point cloud filename, and time stamp
  std::ifstream info_file(keysFilename);
  if (info_file.bad()) {
    ROS_ERROR_STREAM("Failed to open " << keysFilename);
    return false;
  }
  ROS_INFO_STREAM("5");
  std::string keyStr, pcd_filename, timeStr;
  while (info_file.good()) {
    std::getline(info_file, keyStr, ',');
    if (keyStr.empty())
      break;
    key = gtsam::Symbol(std::stoull(keyStr));
    std::getline(info_file, pcd_filename, ',');
    PointCloud::Ptr pc(new PointCloud);
    if (pcl::io::loadPCDFile(pcd_filename, *pc) == -1) {
      ROS_ERROR_STREAM("Failed to load point cloud " << pcd_filename << " from "
                                                     << absFilename);
      return false;
    }
    ROS_INFO_STREAM("Loaded point cloud " << pcd_filename);
    keyed_scans[key] = pc;
    std::getline(info_file, timeStr);
    ros::Time t;
    t.fromNSec(std::stol(timeStr));
    keyed_stamps[key] = t;
    // stamps_keyed_[t] = key ;
  }
  ROS_INFO_STREAM("6");
  // Increment key to be ready for more scans
  key = key + 1;
  ROS_INFO("Restored all point clouds.");
  info_file.close();

  // TODO load edges
  // if (!odometryEdgesFilename.empty()) {
  //   std::ifstream edge_file(odometryEdgesFilename);
  //   if (edge_file.bad()) {
  //     ROS_ERROR_STREAM("Failed to open " << odometryEdgesFilename);
  //     return false;
  //   }
  //   std::string edgeStr;
  //   while (edge_file.good()) {
  //     Edge edge;
  //     std::getline(edge_file, edgeStr, ',');
  //     if (edgeStr.empty())
  //       break;
  //     edge.first = static_cast<gtsam::Key>(std::stoull(edgeStr));
  //     std::getline(edge_file, edgeStr);
  //     edge.second = static_cast<gtsam::Key>(std::stoull(edgeStr));
  //     odometry_edges_.emplace_back(edge);
  //   }
  //   edge_file.close();
  //   ROS_INFO("Restored odometry edges.");
  // }
  ROS_INFO_STREAM("7");
  // if (!loopEdgesFilename.empty()) {
  //   std::ifstream edge_file(loopEdgesFilename);
  //   if (edge_file.bad()) {
  //     ROS_ERROR_STREAM("Failed to open " << loopEdgesFilename);
  //     return false;
  //   }
  //   std::string edgeStr;
  //   while (edge_file.good()) {
  //     Edge edge;
  //     std::getline(edge_file, edgeStr, ',');
  //     if (edgeStr.empty())
  //       break;
  //     edge.first = static_cast<gtsam::Key>(std::stoull(edgeStr));
  //     std::getline(edge_file, edgeStr);
  //     edge.second = static_cast<gtsam::Key>(std::stoull(edgeStr));
  //     loop_edges_.emplace_back(edge);
  //   }
  //   edge_file.close();
  //   ROS_INFO("Restored loop closure edges.");
  // }
  ROS_INFO_STREAM("8");
  // remove all extracted folders
  for (const auto& folder : folders)
    boost::filesystem::remove_all(folder);

  ROS_INFO_STREAM("Successfully loaded pose graph from " << absPath(zipFilename)
                                                         << ".");
  return true;
}
