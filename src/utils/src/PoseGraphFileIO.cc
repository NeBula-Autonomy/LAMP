#pragma once

#include <fstream>

#include <minizip/unzip.h>
#include <minizip/zip.h>

#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>

#include "utils/CommonStructs.h"

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

bool PoseGraph::Save(const std::string& zipFilename) const {
  const std::string path = "pose_graph";
  const boost::filesystem::path directory(path);
  boost::filesystem::create_directory(directory);

  // nfg_.print("PoseGraph::Save: nfg is: ");

  // keys.csv stores factor key, point cloud filename, and time stamp
  std::ofstream keys_file(path + "/keys.csv");
  if (keys_file.bad()) {
    ROS_ERROR("PoseGraph::Save: Failed to write keys file.");
    return false;
  }

  auto zipFile = zipOpen64(zipFilename.c_str(), 0);

  int i = 0;
  for (const auto& entry : keyed_scans) {
    keys_file << gtsam::Key(entry.first) << ",";
    // save point cloud as binary PCD file
    const std::string pcd_filename = path + "/pc_" + std::to_string(i) + ".pcd";
    pcl::io::savePCDFile(pcd_filename, *entry.second, true);
    writeFileToZip(zipFile, pcd_filename);
    ROS_INFO("PoseGraph::Save: Saved point cloud %i/%lu.",
             i + 1,
             keyed_scans.size());
    keys_file << pcd_filename << ",";
    if (!values_.exists(entry.first)) {
      ROS_WARN("PoseGraph::Save: Key %lu associated with a scan does not exist "
               "in values.",
               entry.first);
      return false;
    }
    keys_file << keyed_stamps.at(entry.first).toNSec() << "\n";
    ++i;
  }
  keys_file.close();
  writeFileToZip(zipFile, path + "/keys.csv");

  // convert pose graph to message and save to file
  auto pg_msg = ToMsg();

  rosbag::Bag bag;
  bag.open(path + "/pose_graph.bag", rosbag::bagmode::Write);
  bag.write("pose_graph", ros::Time::now(), *pg_msg);
  bag.close();
  writeFileToZip(zipFile, path + "/pose_graph.bag");

  zipClose(zipFile, 0);
  boost::filesystem::remove_all(directory);
  ROS_INFO_STREAM("Successfully saved pose graph to " << absPath(zipFilename)
                                                      << ".");
  return true;
}

bool PoseGraph::Load(const std::string& zipFilename,
                     const std::string& pose_graph_topic_name) {
  const std::string absFilename = absPath(zipFilename);
  auto zipFile = unzOpen64(zipFilename.c_str());
  // TODO: Storing current key before loading graph to set key to this after
  // stored_key = key;
  if (!zipFile) {
    ROS_ERROR_STREAM("PoseGraph::Load: Failed to open zip file "
                     << absFilename);
    return false;
  }

  unz_global_info64 oGlobalInfo;
  int err = unzGetGlobalInfo64(zipFile, &oGlobalInfo);
  std::vector<std::string> files; // files to be extracted

  std::string keysFilename{""}, pgFilename{""};

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
      if (files.back().find("keys.csv") != std::string::npos) {
        keysFilename = files.back();
      } else if (files.back().find("pose_graph.bag") != std::string::npos) {
        pgFilename = files.back();
      }
      err = unzGoToNextFile(zipFile);
    }
  }
  if (keysFilename.empty()) {
    ROS_ERROR_STREAM("PoseGraph::Load: Could not find keys.csv in "
                     << absFilename);
    return false;
  }

  // extract files
  int i = 1;
  std::vector<boost::filesystem::path> folders;
  for (const auto& filename : files) {
    if (unzLocateFile(zipFile, filename.c_str(), 0) != UNZ_OK) {
      ROS_ERROR_STREAM("PoseGraph::Load: Could not locate file "
                       << filename << " from " << absFilename);
      return false;
    }
    if (unzOpenCurrentFile(zipFile) != UNZ_OK) {
      ROS_ERROR_STREAM("PoseGraph::Load: Could not open file "
                       << filename << " from " << absFilename);
      return false;
    }
    unz_file_info64 oFileInfo;
    if (unzGetCurrentFileInfo64(zipFile, &oFileInfo, 0, 0, 0, 0, 0, 0) !=
        UNZ_OK) {
      ROS_ERROR_STREAM(
          "PoseGraph::Load: Could not determine file size of entry "
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
      ROS_ERROR_STREAM("PoseGraph::Load: Could not create file "
                       << filename << " for extraction.");
      return false;
    }
    if (size > 0) {
      os.write(buf, size);
      os.flush();
    } else {
      ROS_ERROR_STREAM("PoseGraph::Load: Entry "
                       << filename << " from " << absFilename << " is empty.");
    }
    os.close();
    delete[] buf;
    ROS_INFO_STREAM("PoseGraph::Load: Extracted file "
                    << i << "/" << (int)files.size() << " -- " << filename);
    ++i;
  }
  unzClose(zipFile);

  // info_file stores factor key, point cloud filename, and time stamp
  std::ifstream info_file(keysFilename);
  if (info_file.bad()) {
    ROS_ERROR_STREAM("PoseGraph::Load: Failed to open " << keysFilename);
    return false;
  }
  std::string keyStr, pcd_filename, timeStr;
  while (info_file.good()) {
    std::getline(info_file, keyStr, ',');
    if (keyStr.empty())
      break;
    key = gtsam::Symbol(std::stoull(keyStr));
    std::getline(info_file, pcd_filename, ',');
    PointCloud::Ptr pc(new PointCloud);
    if (pcl::io::loadPCDFile(pcd_filename, *pc) == -1) {
      ROS_ERROR_STREAM("PoseGraph::Load: Failed to load point cloud "
                       << pcd_filename << " from " << absFilename);
      return false;
    }
    ROS_INFO_STREAM("PoseGraph::Load: Loaded point cloud " << pcd_filename);
    keyed_scans[key] = pc;
    std::getline(info_file, timeStr);
    ros::Time t;
    t.fromNSec(std::stol(timeStr));
    keyed_stamps[key] = t;
  }
  // Increment key to be ready for more scans
  key = key + 1;
  ROS_INFO("PoseGraph::Load: Restored all %lu point clouds.",
           keyed_scans.size());
  info_file.close();

  rosbag::Bag bag;
  bag.open(pgFilename);
  std::string topic = pose_graph_topic_name;
  if (topic.empty())
    topic = "pose_graph";
  rosbag::View view(bag, rosbag::TopicQuery(topic));
  GraphMsgPtr pg_msg = nullptr;
  // find last pose graph message with desired topic name
  for (const auto& mi : view) {
    GraphMsgPtr current_msg = mi.instantiate<pose_graph_msgs::PoseGraph>();
    if (current_msg != nullptr)
      pg_msg = current_msg;
  }
  if (pg_msg == nullptr) {
    ROS_ERROR_STREAM("Could not read pose graph message from " << pgFilename);
    return false;
  }
  this->UpdateFromMsg(pg_msg);
  bag.close();

  // remove all extracted folders
  for (const auto& folder : folders)
    boost::filesystem::remove_all(folder);

  ROS_INFO_STREAM("Successfully loaded pose graph from " << absPath(zipFilename)
                                                         << ".");
  return true;
}
