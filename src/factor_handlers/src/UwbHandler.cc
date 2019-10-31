/*
 * Copyright Notes
 *
 * Authors: Nobuhiro Funabiki   (nobuhiro.funabiki@jpl.nasa.gov)
*/

#include <factor_handlers/UwbHandler.h>

UwbHandler::UwbHandler() {}

UwbHandler::~UwbHandler() {}

bool UwbHandler::Initialize (const ros::NodeHandle& n) {
    name_ = ros::names::append(n.getNamespace(), "OdometryHandler");
    if (!LoadParameters(n)) {
        ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
        return false;
    }
    if (!RegisterCallbacks(n)) {
        ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
        return false;
    }    
    return true;
}

bool UwbHandler::LoadParameters(const ros::NodeHandle& n) {
    return true;
}

bool UwbHandler::RegisterCallbacks(const ros::NodeHandle& n) {
    ros::NodeHandle nl(n);
    // Subscriber
    uwb_factor_sub_ = nl.subscribe("uwb_factor", 10, &UwbHandler::UwbFactorCallback, this);
    return true;
}


FactorData* UwbHandler::GetData() {
    // UwbData* output = new UwbData(uwb_data_);
    UwbData* output = &uwb_data_;
    if (uwb_data_.factors.size() != 0) {
        output->b_has_data = true;
    }
    else {
        output->b_has_data = false;
    }
    // delete output;
    return output;
}


void UwbHandler::UwbFactorCallback(const pose_graph_msgs::PoseGraph::ConstPtr& msg) {

}
