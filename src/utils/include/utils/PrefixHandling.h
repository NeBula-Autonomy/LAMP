/*
 * Copyright Notes
 *
 * Authors:
 * Alex Stephens       (alex.stephens@jpl.nasa.gov)
 */

#ifndef PREFIX_HANDLING_H
#define PREFIX_HANDLING_H

#include <string>
#include <map>

#include <gtsam/inference/Symbol.h>

namespace utils {

  // Base station
  const std::string LAMP_BASE_PREFIX = "base";
  const gtsam::Symbol LAMP_BASE_INITIAL_KEY('z', 0);

  // UWB
  const char UWB_PREFIX = 'u';

  // Define prefixes for ALL VALID ROBOTS in this file
  const std::map<std::string, char> ROBOT_PREFIXES = {
      {"husky1",  'a'},
      {"husky2",  'b'},
      {"husky3",  'c'},
      {"husky4",  'f'},
      {"telemax1",'d'},
      {"robot",   'e'}
  };

  const std::map<std::string, char> ARTIFACT_PREFIXES = {
      {"husky1",  'l'},
      {"husky2",  'm'},
      {"husky3",  'n'},
      {"husky4",  'q'},
      {"telemax1",'o'},
      {"robot",   'p'}
  };

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
    
    if (ROBOT_PREFIXES.find(robot) != ROBOT_PREFIXES.end()) {
      return 0; 
    }

    return ROBOT_PREFIXES.at(robot);
  }

  // Get the artifact prefix for the given robot
  inline unsigned char GetArtifactPrefix(std::string robot) {
    
    if (ARTIFACT_PREFIXES.find(robot) != ARTIFACT_PREFIXES.end()) {
      return 0; 
    }

    return ARTIFACT_PREFIXES.at(robot);
  }

  // For a given node namespace (e.g. /husky1/lamp_pgo), returns the parameter namespace that should be used ("base" or "robot")
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
