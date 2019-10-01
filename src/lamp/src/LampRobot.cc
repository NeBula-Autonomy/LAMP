/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */


// Includes
#include <lamp/LampRobot.h>

// #include <math.h>
// #include <ctime>

namespace pu = parameter_utils;
namespace gu = geometry_utils;

// Constructor (if there is override)
LampRobot::LampRobot() {}

//Destructor
LampRobot::~LampRobot() {}

// Initialization - override for robot specific setup
LampRobot::Initialize() {}

// Check for data from all of the handlers
bool LampRobot::CheckHandlers() {

    odometry_handler_.GetData();
    artifact_handler_.GetData();

}