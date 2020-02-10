#include "pose_graph_tools/PoseGraphToolsLib.h"

namespace pose_graph_tools
{

PoseGraphToolsLib::PoseGraphToolsLib(void)
{
}

PoseGraphToolsLib::~PoseGraphToolsLib(void)
{
}

std::string PoseGraphToolsLib::bashColor(const color& c) {
  std::stringstream text;
  switch (c) {
  case red:
    text << "\033[1;31m";
    break;
  case green:
    text << "\033[1;32m";
    break;
  case yellow:
    text << "\033[1;33m";
    break;
  case blue:
    text << "\033[1;34m";
    break;
  case magenta:
    text << "\033[1;35m";
    break;
  case cyan:
    text << "\033[1;36m";
    break;
  case white:
    text << "\033[1;37m";
    break;
  default:
    text << "\033[1;37m";
  }
  return text.str();
}

std::string PoseGraphToolsLib::restartWhite(void) {
  std::stringstream text;
  text << "\033[1;37m\033[0m";
  return text.str();
}

std::string PoseGraphToolsLib::print(const std::string& _name,
                                     const std::string& _txt,
                                     const color& _color) {
  std::stringstream text;
  text << print(_name, _color) << " \e[1m" << _txt << "\e[21m" << restartWhite();
  ROS_INFO("%s",text.str().c_str());
  return text.str();
}

std::string PoseGraphToolsLib::print(const std::string& msg, const color& c = white) {
  std::stringstream text;
  text << bashColor(c) << msg << "\033[0m" << restartWhite();
  return text.str();
}
std::string PoseGraphToolsLib::print(const float& msg, const color& c = white) {
  std::stringstream text;
  text << msg << restartWhite();
  return text.str();
}



void simple_do_once()
{
    
}

}

// PoseGraphToolsLib Public API