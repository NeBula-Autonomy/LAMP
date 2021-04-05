#pragma once
#include <cstdint>
#include <map>
#include <pcl/common/colors.h>
#include <pcl/impl/point_types.hpp>
#include <string>
const std::map<unsigned char, pcl::RGB> ROBOT_COLOR = {
    {'a', pcl::GlasbeyLUT::at(0)},  //    {"husky1",  'a'},
    {'b', pcl::GlasbeyLUT::at(1)},  //      {"husky2",  'b'},
    {'c', pcl::GlasbeyLUT::at(2)},  //      {"husky3",  'c'},
    {'f', pcl::GlasbeyLUT::at(3)},  //      {"husky4",  'f'},
    {'d', pcl::GlasbeyLUT::at(4)},  //      {"telemax1",'d'},
    {'e', pcl::GlasbeyLUT::at(5)},  //      {"robot",   'e'},
    {'g', pcl::GlasbeyLUT::at(0)},  //      {"spot1",   'g'},
    {'h', pcl::GlasbeyLUT::at(1)},  //      {"spot2",   'h'},
    {'i', pcl::GlasbeyLUT::at(8)},  //      {"handheld1",   'i'},
    {'j', pcl::GlasbeyLUT::at(9)}}; //{ "xmaxx1", 'j' }
