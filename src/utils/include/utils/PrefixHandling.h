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

namespace utils {

  const gtsam::Symbol LAMP_BASE_INITIAL_KEY('z', 0);

  const std::map<std::string, char> ROBOT_PREFIX = {
      {"husky1",  'a'},
      {"husky2",  'b'},
      {"husky3",  'c'},
      {"telemax1",'d'},
      {"robot",   'e'},
      {"husky4",  'f'}
  };

  const std::map<std::string, char> ROBOT_ARTIFACT_PREFIX = {
      {"husky1",  'l'},
      {"husky2",  'm'},
      {"husky3",  'n'},
      {"telemax1",'o'},
      {"robot",   'p'},
      {"husky4",  'q'}
  };

  inline bool IsRobotPrefix(unsigned char c) {
    for (auto k : ROBOT_PREFIX) {  
      if (k.second == c) {
        return true;
      }
    }
    return false;
  }

  inline bool IsRobotArtifactPrefix(unsigned char c) {
    for (auto k : ROBOT_ARTIFACT_PREFIX) {  
      if (k.second == c) {
        return true;
      }
    }
    return false;
  }

  inline unsigned char GetRobotPrefix(std::string robot) {
    
    if (ROBOT_PREFIX.find(robot) != ROBOT_PREFIX.end()) {
      return 0; 
    }

    return ROBOT_PREFIX.at(robot);
  }

  inline unsigned char GetRobotArtifactPrefix(std::string robot) {
    
    if (ROBOT_PREFIX.find(robot) != ROBOT_PREFIX.end()) {
      return 0; 
    }

    return ROBOT_ARTIFACT_PREFIX.at(robot);
  }

} // namespace utils



#endif
