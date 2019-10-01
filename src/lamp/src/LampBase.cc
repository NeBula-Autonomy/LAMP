/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */


// Includes
#include <lamp/LampBase.h>

// #include <math.h>
// #include <ctime>

namespace pu = parameter_utils;
namespace gu = geometry_utils;

// Constructor
LampBase::LampBase()
  : example_variable_(3.14159),
    variable_2_(2),
    example_boolean_(false) {
     // any other things on construction 
    }

// Destructor
LampBase::~LampBase() {}

// Initialization
LampBase::Initialize() {

  LoadParameters();
  CreatePublishers();
  InitializeHandlers();

}

// Load Parameters
LampBase::LoadParameters() {}

// Create Publishers
LampBase::CreatePublishers() {}


