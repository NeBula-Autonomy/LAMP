#pragma once
#include <cstdint>
#include <map>
#include <pcl/common/colors.h>
#include <pcl/impl/point_types.hpp>
#include <string>
const std::map<unsigned char, pcl::RGB> ROBOT_COLOR = {
    {'a', pcl::GlasbeyLUT::at(0)},   //    {"husky1",  'a'},
    {'b', pcl::GlasbeyLUT::at(1)},   //      {"husky2",  'b'},
    {'c', pcl::GlasbeyLUT::at(2)},   //      {"husky3",  'c'},
    {'d', pcl::GlasbeyLUT::at(3)},   //      {"husky4",  'd'},
    {'e', pcl::GlasbeyLUT::at(4)},   //      {"spot1",'e'},
    {'f', pcl::GlasbeyLUT::at(5)},   //      {"spot2",   'f'},
    {'g', pcl::GlasbeyLUT::at(6)},   //      {"spot3",   'g'},
    {'h', pcl::GlasbeyLUT::at(9)},   //      {"spot4",   'h'},
    {'i', pcl::GlasbeyLUT::at(10)},  //      {"aquila1",   'i'},
    {'j', pcl::GlasbeyLUT::at(10)},  //      {"aquila2", 'j' },
    {'k', pcl::GlasbeyLUT::at(11)},  //      {"kaist1", 'k' },
    {'l', pcl::GlasbeyLUT::at(12)},  //      {"zoe1", 'l' },
    {'m', pcl::GlasbeyLUT::at(12)},  //      {"zoe2", 'm' },
    {'x', pcl::GlasbeyLUT::at(13)}}; //      {"xmaxx1", 'x' }
