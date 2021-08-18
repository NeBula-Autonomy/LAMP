/*
 * Copyright Notes
 *
 * Authors:
 * Alex Stephens       (alex.stephens@jpl.nasa.gov)
 */

#ifndef PREFIX_HANDLING_H
#define PREFIX_HANDLING_H

#include <map>
#include <ros/ros.h>
#include <string>

#include <gtsam/inference/Symbol.h>

namespace utils {

  // Base station
  const std::string LAMP_BASE_PREFIX = "base";
  const gtsam::Symbol GTSAM_ERROR_SYMBOL('x', 9999);

  // UWB
  const char UWB_PREFIX = 'u';

  // Define prefixes for ALL VALID ROBOTS in this file
  const std::map<std::string, char> ROBOT_PREFIXES = {{"husky1", 'a'},
                                                      {"husky2", 'b'},
                                                      {"husky3", 'c'},
                                                      {"husky4", 'd'},
                                                      {"spot1", 'e'},
                                                      {"spot2", 'f'},
                                                      {"spot3", 'g'},
                                                      {"spot4", 'h'},
                                                      {"aquila1", 'i'},
                                                      {"aquila2", 'j'},
                                                      {"kasit1", 'k'},
                                                      {"zoe1", 'l'},
                                                      {"zoe2", 'm'},
                                                      {"xmaxx1", 'x'},
                                                      {"base", 'z'}};

  const std::map<std::string, char> ARTIFACT_PREFIXES = {{"husky1", 'A'},
                                                         {"husky2", 'B'},
                                                         {"husky3", 'C'},
                                                         {"husky4", 'D'},
                                                         {"spot1", 'E'},
                                                         {"spot2", 'F'},
                                                         {"spot3", 'G'},
                                                         {"spot4", 'H'},
                                                         {"aquila1", 'I'},
                                                         {"aquila2", 'J'},
                                                         {"kasit1", 'K'},
                                                         {"zoe1", 'L'},
                                                         {"zoe2", 'M'},
                                                         {"xmaxx1", 'X'}};

  // ---------------------------------------------------------
  //                    Query functions
  // ---------------------------------------------------------


  // Checks if the character is a robot node prefix;
  inline bool IsRobotPrefix(unsigned char c) {
    for (auto k : ROBOT_PREFIXES) {  
      if (k.second == c) {
        return true;
      }
    }
    return false;
  }

  // Checks if the character is an artifact prefix;
  inline bool IsArtifactPrefix(unsigned char c) {
    for (auto k : ARTIFACT_PREFIXES) {
      if (k.second == c) {
        return true;
      }
    }
    return false;
  }

  // Checks if the character is an artifact or UWB prefix;
  inline bool IsSpecialSymbol(unsigned char c) {
    return IsArtifactPrefix(c) || (c == UWB_PREFIX);
  }

  // Get the prefix for the given robot
  inline unsigned char GetRobotPrefix(std::string robot) {
    if (!ROBOT_PREFIXES.count(robot)) {
      return 0;
    }

    return ROBOT_PREFIXES.at(robot);
  }

  // Get the artifact prefix for the given robot
  inline unsigned char GetArtifactPrefix(std::string robot) {
    if (!ARTIFACT_PREFIXES.count(robot)) {
      return 0;
    }

    return ARTIFACT_PREFIXES.at(robot);
  }

  // For a given node namespace (e.g. /husky1/lamp_pgo), returns the parameter
  // namespace that should be used ("base" or "robot")
  inline std::string GetParamNamespace(std::string ns) {
    if (ns.find("base") != std::string::npos) {
      return "base";
    }

    for (auto pair : ROBOT_PREFIXES) {
      if (ns.find(pair.first) != std::string::npos) {
        return "robot";
      }
    }

    ROS_ERROR_STREAM("Namespace not recognized as base station or robot");
    return "";
  }

  // ---------------------------------------------------------
  //                    Key comparison functions
  // ---------------------------------------------------------

  // Checks if two keys come from the same robot
  inline bool IsKeyFromSameRobot(gtsam::Symbol key1, gtsam::Symbol key2) {
    if (!utils::IsRobotPrefix(key1.chr())) {
      ROS_ERROR_STREAM(gtsam::DefaultKeyFormatter(key1)
                       << " is not a valid robot key.");
      return false;
    } else if (!utils::IsRobotPrefix(key2.chr())) {
      ROS_ERROR_STREAM(gtsam::DefaultKeyFormatter(key2)
                       << " is not a valid robot key.");
      return false;
    }

    return key1.chr() == key2.chr();
  }

  // ---------------------------------------------------------
  //                    Get full vectors
  // ---------------------------------------------------------

  // Get all the robot prefixes
  inline std::vector<char> GetAllRobotPrefixes() {
    std::vector<char> output;
    for (auto p : ROBOT_PREFIXES) {
      output.push_back(p.second);
    }
    return output;
  }

  // Get all the artifact prefixes
  inline std::vector<char> GetAllArtifactPrefixes() {
    std::vector<char> output;
    for (auto p : ARTIFACT_PREFIXES) {
      output.push_back(p.second);
    }
    return output;
  }

  // Get all the artifact and UWB prefixes
  inline std::vector<char> GetAllSpecialSymbols() {
    std::vector<char> output = GetAllArtifactPrefixes();
    output.push_back(UWB_PREFIX);
    return output;
  }

} // namespace utils

#endif
