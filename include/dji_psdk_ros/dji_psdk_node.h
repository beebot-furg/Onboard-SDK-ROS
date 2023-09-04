#ifndef __DJI_PSDK_NODE_HH__
#define __DJI_PSDK_NODE_HH__

// Header include
#include <ros/ros.h>
#include <memory>
#include <string>

// PSDK include
#include "modules/application.hpp"
#include "dji_aircraft_info.h"
#include "dji_platform.h"

//! ROS standard msgs
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <nmea_msgs/Sentence.h>

/*! services */
//flight control services
#include <dji_osdk_ros/GetDroneType.h>
#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/SetJoystickMode.h>
#include <dji_osdk_ros/JoystickAction.h>
#include <dji_osdk_ros/SetGoHomeAltitude.h>
#include <dji_osdk_ros/GetGoHomeAltitude.h>
#include <dji_osdk_ros/SetHomePoint.h>
#include <dji_osdk_ros/SetCurrentAircraftLocAsHomePoint.h>
#include <dji_osdk_ros/SetLocalPosRef.h>
#include <dji_osdk_ros/SetAvoidEnable.h>
#include <dji_osdk_ros/GetAvoidEnable.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>
#include <dji_osdk_ros/KillSwitch.h>
#include <dji_osdk_ros/EmergencyBrake.h>
//Gimbal control services
#include <dji_osdk_ros/GimbalAction.h>

//Camera control services
#include <dji_osdk_ros/CameraEV.h>
#include <dji_osdk_ros/CameraShutterSpeed.h>
#include <dji_osdk_ros/CameraAperture.h>
#include <dji_osdk_ros/CameraISO.h>
#include <dji_osdk_ros/CameraFocusPoint.h>
#include <dji_osdk_ros/CameraTapZoomPoint.h>
#include <dji_osdk_ros/CameraSetZoomPara.h>
#include <dji_osdk_ros/CameraZoomCtrl.h>
#include <dji_osdk_ros/CameraStartShootBurstPhoto.h>
#include <dji_osdk_ros/CameraStartShootAEBPhoto.h>
#include <dji_osdk_ros/CameraStartShootSinglePhoto.h>
#include <dji_osdk_ros/CameraStartShootIntervalPhoto.h>
#include <dji_osdk_ros/CameraStopShootPhoto.h>
#include <dji_osdk_ros/CameraRecordVideoAction.h>

//HMS services
#include <dji_osdk_ros/GetHMSData.h>
//mfio services
#include <dji_osdk_ros/MFIO.h>
//MOP services
#include <dji_osdk_ros/SendMobileData.h>
#include <dji_osdk_ros/SendPayloadData.h>
//mission services
#include <dji_osdk_ros/MissionStatus.h>
#include <dji_osdk_ros/MissionWpUpload.h>
#include <dji_osdk_ros/MissionWpAction.h>
#include <dji_osdk_ros/MissionWpGetSpeed.h>
#include <dji_osdk_ros/MissionWpSetSpeed.h>
#include <dji_osdk_ros/MissionWpGetInfo.h>
#include <dji_osdk_ros/MissionHpUpload.h>
#include <dji_osdk_ros/MissionHpAction.h>
#include <dji_osdk_ros/MissionHpGetInfo.h>
#include <dji_osdk_ros/MissionHpUpdateYawRate.h>
#include <dji_osdk_ros/MissionHpResetYaw.h>
#include <dji_osdk_ros/MissionHpUpdateRadius.h>
//battery services
#include <dji_osdk_ros/GetWholeBatteryInfo.h>
#include <dji_osdk_ros/GetSingleBatteryDynamicInfo.h>

//waypointV2.0 services
#include <dji_osdk_ros/InitWaypointV2Setting.h>
#include <dji_osdk_ros/UploadWaypointV2Mission.h>
#include <dji_osdk_ros/UploadWaypointV2Action.h>
#include <dji_osdk_ros/DownloadWaypointV2Mission.h>
#include <dji_osdk_ros/StartWaypointV2Mission.h>
#include <dji_osdk_ros/StopWaypointV2Mission.h>
#include <dji_osdk_ros/PauseWaypointV2Mission.h>
#include <dji_osdk_ros/ResumeWaypointV2Mission.h>
#include <dji_osdk_ros/GenerateWaypointV2Action.h>
#include <dji_osdk_ros/SetGlobalCruisespeed.h>
#include <dji_osdk_ros/GetGlobalCruisespeed.h>
#include <dji_osdk_ros/SubscribeWaypointV2Event.h>
#include <dji_osdk_ros/SubscribeWaypointV2State.h>

#ifdef ADVANCED_SENSING
#include <dji_osdk_ros/SetupCameraH264.h>
#include <dji_osdk_ros/SetupCameraStream.h>
#include <dji_osdk_ros/Stereo240pSubscription.h>
#include <dji_osdk_ros/StereoDepthSubscription.h>
#include <dji_osdk_ros/StereoVGASubscription.h>
#include <dji_osdk_ros/GetM300StereoParams.h>
#endif

/*! msgs */
#include <dji_osdk_ros/Gimbal.h>
#include <dji_osdk_ros/MobileData.h>
#include <dji_osdk_ros/PayloadData.h>
#include <dji_osdk_ros/FlightAnomaly.h>
#include <dji_osdk_ros/VOPosition.h>
#include <dji_osdk_ros/FCTimeInUTC.h>
#include <dji_osdk_ros/GPSUTC.h>

//waypointV2.0
#include <dji_osdk_ros/WaypointV2.h>
#include <dji_osdk_ros/WaypointV2Action.h>
#include <dji_osdk_ros/WaypointV2AircraftControlActuator.h>
#include <dji_osdk_ros/WaypointV2AircraftControlActuatorFlying.h>
#include <dji_osdk_ros/WaypointV2AircraftControlActuatorRotateHeading.h>
#include <dji_osdk_ros/WaypointV2AssociateTrigger.h>
#include <dji_osdk_ros/WaypointV2CameraActuator.h>
#include <dji_osdk_ros/WaypointV2CameraActuatorFocusParam.h>
#include <dji_osdk_ros/WaypointV2CameraActuatorFocalLengthParam.h>
#include <dji_osdk_ros/WaypointV2Config.h>
#include <dji_osdk_ros/WaypointV2GimbalActuator.h>
#include <dji_osdk_ros/WaypointV2GimbalActuatorRotationParam.h>
#include <dji_osdk_ros/WaypointV2InitSetting.h>
#include <dji_osdk_ros/WaypointV2IntervalTrigger.h>
#include <dji_osdk_ros/WaypointV2ReachpointTrigger.h>
#include <dji_osdk_ros/WaypointV2SampleReachPointTrigger.h>
#include <dji_osdk_ros/WaypointV2TrajectoryTrigger.h>
#include <dji_osdk_ros/WaypointV2MissionEventPush.h>
#include <dji_osdk_ros/WaypointV2MissionStatePush.h>

// #define C_EARTH (double)6378137.0
// #define C_PI (double)3.141592653589793
// #define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
// #define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))
// const int WAIT_TIMEOUT = 10;
// const int FLIGHT_CONTROL_WAIT_TIMEOUT = 1;

// Declaration
namespace dji_psdk_ros
{
  class PSDKNode
  {
    public:
      PSDKNode();

      ~PSDKNode();

      // bool initGimbalModule();
      // bool initCameraModule();
      void initService();
      // bool initTopic();
      // bool initDataSubscribeFromFC();
      // bool cleanUpSubscribeFromFC();
    protected:
      /*! services */

      /*! for general */
      ros::ServiceServer get_drone_type_server_;
      ros::ServiceServer get_whole_battery_info_server_;


    protected:
      /*! for general */
      bool getDroneTypeCallback(dji_osdk_ros::GetDroneType::Request &request,
                                dji_osdk_ros::GetDroneType::Response &response);
      
      bool getWholeBatteryInfoCallback(dji_osdk_ros::GetWholeBatteryInfo::Request& request,
                                       dji_osdk_ros::GetWholeBatteryInfo::Response& reponse);

    private:
      ros::NodeHandle nh_;

      Application *application;
  };
}
#endif // __DJI_PSDK_NODE_HH__

