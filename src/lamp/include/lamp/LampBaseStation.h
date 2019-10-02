/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#ifndef LAMP_BASE_STATION_H
#define LAMP_BASE_STATION_H

// Includes 
#include <lamp/LampBase.h>


// Services

// Class Definition
class LampBaseStation : public LampBase {
  public:
    // typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    // Constructor
    LampBaseStation();

    // Destructor
    ~LampBaseStation();    


    // Override base class functions where needed 
    bool Initialize();


  private:
    // Overwrite base classs functions where needed


    // Add new functions as needed


    // Add new variables as needed


};


#endif