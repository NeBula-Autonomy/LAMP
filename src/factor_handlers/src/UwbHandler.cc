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
    ResetFactorData();
    return true;
}

bool UwbHandler::RegisterCallbacks(const ros::NodeHandle& n) {
    ros::NodeHandle nl(n);
    // Subscriber
    uwb_factor_sub_ = nl.subscribe("uwb_factors", 10, &UwbHandler::UwbFactorCallback, this);
    return true;
}


std::shared_ptr<FactorData> UwbHandler::GetData() {
    std::shared_ptr<UwbData> data_ptr = std::make_shared<UwbData>(uwb_data_);
    if (uwb_data_.factors.size() != 0) {
        data_ptr->b_has_data = true;
    }
    else {
        data_ptr->b_has_data = false;
    }
    // delete output;
    return data_ptr;
}


void UwbHandler::UwbFactorCallback(const pose_graph_msgs::PoseGraph::ConstPtr& msg) {
    ROS_INFO("UwbHandler received UWB range factors from UwbFrontend");
    for (const auto& edge : msg->edges) {
        UwbFactor factor;
        factor.key_from     = edge.key_from;
        factor.key_to       = edge.key_to;
        factor.range        = edge.range;
        factor.range_error  = edge.range_error;
        uwb_data_.factors.emplace_back(factor);
    }
}
