/*
 * @file  roadmap_bridge.h
 * @brief Bridge between ROS and BGL (Boost Graph Library)
 */

#pragma once

#include <boost/shared_ptr.hpp>

#include "roadmap_msgs/Roadmap.h"
#include "roadmap_msgs/roadmap_graph.h"
#include "roadmap_msgs/roadmap_type.h"

namespace roadmap {

/** Convert BGL-style graph to ROS message */
roadmap_msgs::RoadmapPtr toROSMsg(const RoadmapGraph &graph);

/** Convert ROS-style graph to BGL object */
RoadmapGraph fromROSMsg(const roadmap_msgs::Roadmap &msg);
RoadmapGraph fromROSMsg(const roadmap_msgs::RoadmapConstPtr &msg);

} // namespace roadmap
