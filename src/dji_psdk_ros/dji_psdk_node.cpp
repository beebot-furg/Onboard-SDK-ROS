#include "dji_psdk_ros/dji_psdk_node.h"

using namespace dji_psdk_ros;

PSDKNode::PSDKNode()
{
  application = new Application();

  osalHandler = DjiPlatform_GetOsalHandler();

  initService();
}

PSDKNode::~PSDKNode() {}

void PSDKNode::initService() 
{
  ROS_INFO_STREAM("Services startup!");

  get_drone_type_server_ = nh_.advertiseService("get_drone_type", &PSDKNode::getDroneTypeCallback, this);

  //get_whole_battery_info_server_ = nh_.advertiseService("get_whole_battery_info", &VehicleNode::getWholeBatteryInfoCallback, this);
}

bool PSDKNode::getDroneTypeCallback(dji_osdk_ros::GetDroneType::Request &request,
                                       dji_osdk_ros::GetDroneType::Response &response)
{
  ROS_DEBUG("called getDroneTypeCallback");

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "psdk_node");
  PSDKNode psdk_node;

  ros::spin();
  return 0;
}
