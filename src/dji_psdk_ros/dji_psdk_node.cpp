#include "dji_psdk_ros/dji_psdk_node.h"

#include "dji_logger.h"
#include "dji_typedef.h"
#include "dji_fc_subscription.h"
#include "dji_aircraft_info.h"

using namespace dji_psdk_ros;

PSDKNode::PSDKNode()
{
  application = new Application();

  T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

  initService();

  DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                             NULL);
}

PSDKNode::~PSDKNode() {}

void PSDKNode::initService() 
{
  ROS_INFO_STREAM("Services startup!");

  get_drone_type_server_ = nh_.advertiseService("get_drone_type", &PSDKNode::getDroneTypeCallback, this);

  get_whole_battery_info_server_ = nh_.advertiseService("get_whole_battery_info", &PSDKNode::getWholeBatteryInfoCallback, this);
}

bool PSDKNode::getDroneTypeCallback(dji_osdk_ros::GetDroneType::Request &request,
                                       dji_osdk_ros::GetDroneType::Response &response)
{
  ROS_DEBUG("called get Drone Type Callback");

  T_DjiReturnCode djiStat;

  T_DjiDataTimestamp timestamp = {0};
  T_DjiAircraftInfoBaseInfo droneInfo;

  djiStat = DjiAircraftInfo_GetBaseInfo(&droneInfo);
                                                    ;
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      ROS_ERROR("get drone info failed.");
      return false;
  }

  ROS_INFO("drone type: %d.", droneInfo.aircraftType);

  response.drone_type = droneInfo.aircraftType;

  return true;
}

bool PSDKNode::getWholeBatteryInfoCallback(dji_osdk_ros::GetWholeBatteryInfo::Request& request, 
                                           dji_osdk_ros::GetWholeBatteryInfo::Response& response)
{
  ROS_INFO_STREAM("get Whole Battery Info callback");

  T_DjiReturnCode djiStat;
  T_DjiDataTimestamp timestamp = {0};
  T_DjiFcSubscriptionWholeBatteryInfo batteryInfo = {0};

  djiStat = DjiFcSubscription_Init();
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      ROS_ERROR("init data subscription module error.");
      return false;
  }

  

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO,
                                                    (uint8_t *) &batteryInfo,
                                                    sizeof(T_DjiFcSubscriptionWholeBatteryInfo),
                                                    &timestamp);

  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      ROS_ERROR("get value of topic whole battery info error.");
      return false;
  } 
  ROS_INFO("battery whole info: capacity percent = %ld voltage = %ld.",
           batteryInfo.percentage,
           batteryInfo.voltage / 1000);

  // response.battery_whole_info.remainFlyTime  = batteryWholeInfo.remainFlyTime;
  // response.battery_whole_info.goHomeNeedTime = batteryWholeInfo.goHomeNeedTime ;
  // response.battery_whole_info.landNeedTime   = batteryWholeInfo.landNeedTime;
  // response.battery_whole_info.goHomeNeedCapacity = batteryWholeInfo.goHomeNeedCapacity;
  // response.battery_whole_info.landNeedCapacity = batteryWholeInfo.landNeedCapacity ;
  // response.battery_whole_info.safeFlyRadius = batteryWholeInfo.safeFlyRadius;
  // response.battery_whole_info.capacityConsumeSpeed = batteryWholeInfo.capacityConsumeSpeed;
  // response.battery_whole_info.goHomeCountDownState = batteryWholeInfo.goHomeCountDownState;
  // response.battery_whole_info.gohomeCountDownvalue = batteryWholeInfo.gohomeCountDownvalue;
  response.battery_whole_info.voltage = batteryInfo.voltage;
  response.battery_whole_info.batteryCapacityPercentage = batteryInfo.percentage;
  // response.battery_whole_info.lowBatteryAlarmThreshold = batteryWholeInfo.lowBatteryAlarmThreshold;
  // response.battery_whole_info.lowBatteryAlarmEnable = batteryWholeInfo.lowBatteryAlarmEnable;
  // response.battery_whole_info.seriousLowBatteryAlarmThreshold = batteryWholeInfo.seriousLowBatteryAlarmThreshold;
  // response.battery_whole_info.seriousLowBatteryAlarmEnable = batteryWholeInfo.seriousLowBatteryAlarmEnable;

  // response.battery_whole_info.batteryState.voltageNotSafety        = batteryWholeInfo.batteryState.voltageNotSafety;
  // response.battery_whole_info.batteryState.veryLowVoltageAlarm     = batteryWholeInfo.batteryState.veryLowVoltageAlarm;
  // response.battery_whole_info.batteryState.LowVoltageAlarm         = batteryWholeInfo.batteryState.LowVoltageAlarm;
  // response.battery_whole_info.batteryState.seriousLowCapacityAlarm = batteryWholeInfo.batteryState.seriousLowCapacityAlarm;
  // response.battery_whole_info.batteryState.LowCapacityAlarm        = batteryWholeInfo.batteryState.LowCapacityAlarm;

  djiStat = DjiFcSubscription_DeInit();
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      ROS_ERROR("Deinit fc subscription error.");
      return false;
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "psdk_node");
  PSDKNode psdk_node;

  ros::spin();
  return 0;
}
