/*
 * Copyright Notes
 *
 * Authors: Nobuhiro Funabiki   (nobuhiro.funabiki@jpl.nasa.gov)
*/

#ifndef UWB_HANDLER_H
#define UWB_HANDLER_H

#include <factor_handlers/LampDataHandlerBase.h>

class UwbHandler : public LampDataHandlerBase {

    public:
        UwbHandler();
        ~UwbHandler();

        bool Initialize (const ros::NodeHandle& n);
    
    private:
        
        bool LoadParameters(const ros::NodeHandle& n);
        bool RegisterCallbacks(const ros::NodeHandle& n);

        // The node's name.
        std::string name_;

};

#endif
