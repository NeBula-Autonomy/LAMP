
/*
 * Copyright Notes
 *
 * Authors: Matteo Palieri      (matteo.palieri@jpl.nasa.gov)
 *          Kamak Ebadi         (kamak.ebadi@jpl.nasa.gov)
 *          Nobuhiro Funabiki   (nobuhiro.funabiki@jpl.nasa.gov)
 */
#ifndef ODOMETRY_HANDLER_H
#define ODOMETRY_HANDLER_H

// Includes
#include <ros/ros.h>
#include <factor_handlers/LampDataHandlerBase.h>

// #include <ros/ros.h>
// #include <gtsam/geometry/Pose3.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <gtsam/geometry/Pose3.h>
// #include <geometry_utils/Transform3.h>

// namespace gu = geometry_utils;
// namespace gr = gu::ros;
// namespace pu = parameter_utils;
// using namespace std; 

class OdometryHandler : public LampDataHandlerBase{
    
    public:
     
        OdometryHandler();
        ~OdometryHandler();        

    protected: 

    private:    
};
    
#endif