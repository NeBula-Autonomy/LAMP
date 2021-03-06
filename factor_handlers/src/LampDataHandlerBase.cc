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

// Destructor
LampDataHandlerBase::~LampDataHandlerBase() {}

// // Main interface call
// FactorData* LampDataHandlerBase::GetData() {
//   // Get the factors
//   FactorData* output_factors = new FactorData(factors_);

//   // Reset factors
//   ResetFactorData();

//   return output_factors;
// }

// // Reset factor data
// void LampDataHandlerBase::ResetFactorData() {
//   // Create empty factors
//   FactorData empty_factors;
//   empty_factors.b_has_data = false;

//   // Reset the factors_variable
//   factors_ = empty_factors;
// }