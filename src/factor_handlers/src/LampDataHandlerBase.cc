/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */


// Includes
#include <factor_handlers/LampDataHandlerBase.h>
#include <parameter_utils/ParameterUtils.h>


// #include <math.h>
// #include <ctime>

namespace pu = parameter_utils;
namespace gu = geometry_utils;

// Constructor
LampDataHandlerBase::LampDataHandlerBase() {}

//Destructor
LampDataHandlerBase::~LampDataHandlerBase() {}

// Initialization
bool LampDataHandlerBase::Initialize(const ros::NodeHandle& n) {}

// Main interface call
FactorData LampDataHandlerBase::GetData() {}
