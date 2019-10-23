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

  const char UWB_PREFIX = 'u';

  const std::map<std::string, char> ROBOT_PREFIX = {
      {"husky1",  'a'},
      {"husky2",  'b'},
      {"husky3",  'c'},
      {"husky4",  'f'},
      {"telemax1",'d'},
      {"robot",   'e'}
  };

  const std::map<std::string, char> ARTIFACT_PREFIX = {
      {"husky1",  'l'},
      {"husky2",  'm'},
      {"husky3",  'n'},
      {"husky4",  'q'},
      {"telemax1",'o'},
      {"robot",   'p'}
  };

  // Checks if the character is a robot node prefix;
  inline bool IsRobotPrefix(unsigned char c) {
    for (auto k : ROBOT_PREFIX) {  
      if (k.second == c) {
        return true;
      }
    }
    return false;
  }

  // Checks if the character is an artifact prefix;
  inline bool IsArtifactPrefix(unsigned char c) {
    for (auto k : ARTIFACT_PREFIX) {  
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

  inline unsigned char GetRobotPrefix(std::string robot) {
    
    if (ROBOT_PREFIX.find(robot) != ROBOT_PREFIX.end()) {
      return 0; 
    }

    return ROBOT_PREFIX.at(robot);
  }

  inline unsigned char GetArtifactPrefix(std::string robot) {
    
    if (ARTIFACT_PREFIX.find(robot) != ARTIFACT_PREFIX.end()) {
      return 0; 
    }

    return ARTIFACT_PREFIX.at(robot);
  }

} // namespace utils



#endif
