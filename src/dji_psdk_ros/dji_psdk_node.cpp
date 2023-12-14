#include "dji_psdk_ros/dji_psdk_node.h"

using namespace dji_psdk_ros;

PSDKNode::PSDKNode()
{
  application = new Application();

  // this need to be made in another class called "LiveView Manager" or "Camera Manager"
  // doing this process here just as a POC
  // T_DjiReturnCode returnCode;
  // returnCode = DjiLiveview_Init();
  // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
  //     ROS_ERROR("Liveview init failed");
  // }

  // streamDecoder = {
  //     {DJI_LIVEVIEW_CAMERA_POSITION_FPV,  (new DJICameraStreamDecoder())},
  //     {DJI_LIVEVIEW_CAMERA_POSITION_NO_1, (new DJICameraStreamDecoder())}
  // };

  initService();
  initTopic();
}

PSDKNode::~PSDKNode() {}

bool PSDKNode::initTopic()
{
  attitude_publisher_ = nh_.advertise<geometry_msgs::QuaternionStamped>("dji_osdk_ros/attitude", 10);

  main_camera_stream_publisher_ = nh_.advertise<sensor_msgs::Image>("dji_osdk_ros/main_camera_images", 10);
  fpv_camera_stream_publisher_ = nh_.advertise<sensor_msgs::Image>("dji_osdk_ros/fpv_camera_images", 10);

  ROS_INFO_STREAM("Topics startup!");
}

void PSDKNode::initService() 
{
  get_drone_type_server_ = nh_.advertiseService("get_drone_type", 
                                                &PSDKNode::getDroneTypeCallback, this);
  get_whole_battery_info_server_ = nh_.advertiseService("get_whole_battery_info",
                                                        &PSDKNode::getWholeBatteryInfoCallback, this);
  setup_camera_stream_server_ = nh_.advertiseService("setup_camera_stream",
                                                     &PSDKNode::setupCameraStreamCallback, this);

  ROS_INFO_STREAM("Services startup!");
}

// static void LiveviewConvertH264ToRgbCallback(E_DjiLiveViewCameraPosition position,
//                                              const uint8_t *buf, uint32_t bufLen)
// {
//   auto deocder = streamDecoder.find(position);
//   if ((deocder != streamDecoder.end()) && deocder->second) {
//       deocder->second->decodeBuffer(buf, bufLen);
//   }
// }

// bool PSDKNode::startFPVCameraStream(CameraImageCallback callback, void *userData)
// {
//     auto decoder = streamDecoder.find(DJI_LIVEVIEW_CAMERA_POSITION_FPV);

//     if ((decoder != streamDecoder.end()) && decoder->second) {
//       decoder->second->init();
//       decoder->second->registerCallback(callback, userData);

//       return !static_cast<bool>(DjiLiveview_StartH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_FPV, 
//                                                             DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT,
//                                                             LiveviewConvertH264ToRgbCallback));
//     } 
//     else {
//       return false;
//     }
// }

// bool PSDKNode::stopFPVCameraStream()
// {
//     // T_DjiReturnCode returnCode;

//     // returnCode = DjiLiveview_StopH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_FPV, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
//     // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//     //     return returnCode;
//     // }

//     auto deocder = streamDecoder.find(DJI_LIVEVIEW_CAMERA_POSITION_FPV);
//     if ((deocder != streamDecoder.end()) && deocder->second) {
//         deocder->second->cleanup();
//     }

//     return true;
// }

// bool PSDKNode::startMainCameraStream(CameraImageCallback callback, void *userData)
// {
//     auto decoder = streamDecoder.find(DJI_LIVEVIEW_CAMERA_POSITION_NO_1);

//     if ((decoder != streamDecoder.end()) && decoder->second) {
//         decoder->second->init();
//         decoder->second->registerCallback(callback, userData);

//         return true;
//     } else {
//         return false;
//     }
// }

// bool PSDKNode::stopMainCameraStream()
// {
//     // T_DjiReturnCode returnCode;

//     // returnCode = DjiLiveview_StopH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_NO_1, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
//     // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//     //     return returnCode;
//     // }

//     auto deocder = streamDecoder.find(DJI_LIVEVIEW_CAMERA_POSITION_NO_1);
//     if ((deocder != streamDecoder.end()) && deocder->second) {
//         deocder->second->cleanup();
//     }

//     return true;
// }

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

  T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
  T_DjiReturnCode djiStat;
  T_DjiDataTimestamp timestamp = {0};
  T_DjiFcSubscriptionWholeBatteryInfo batteryInfo = {0};

  DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                   NULL);

  osalHandler->TaskSleepMs(1000);

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

static void DjiUser_ShowRgbImageCallback(CameraRGBImage img, void *userData)
{
}


bool PSDKNode::setupCameraStreamCallback(dji_osdk_ros::SetupCameraStream::Request& request,
                                         dji_osdk_ros::SetupCameraStream::Response& response)
{
  T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

  char fpvName[] = "FPV_CAM";
  char mainName[] = "MAIN_CAM";
  char viceName[] = "VICE_CAM";
  char topName[] = "TOP_CAM";

  auto *liveviewSample = new LiveviewSample();
  
  ROS_INFO_STREAM("called camera Stream Callback");

  if(request.cameraType == request.FPV_CAM)
  {
    if(request.start == 1)
    {
      char name[] = "FPV_CAM";
      response.result = liveviewSample->StartFpvCameraStream(&DjiUser_ShowRgbImageCallback, &fpvName);
    }
    else
    {
      response.result = liveviewSample->StopFpvCameraStream();
    }
  }
  else if(request.cameraType == request.MAIN_CAM)
  {
    if(request.start == 1)
    {
      response.result = liveviewSample->StartMainCameraStream(&publishMainCameraImage, &mainName);
    }
    else
    {
      response.result = liveviewSample->StopMainCameraStream();
    }
  }

  char isQuit;
  while (true) {
    cin >> isQuit;
    if (isQuit == 'q' || isQuit == 'Q') {
        break;
    }
  }

  osalHandler->TaskSleepMs(2000);

  return response.result;
}

void PSDKNode::publishFPVCameraImage(CameraRGBImage rgbImg, void* userData)
{
  ROS_INFO("FPV CALLBACK");

  // sensor_msgs::Image img;
  // img.height = rgbImg.height;
  // img.width = rgbImg.width;
  // img.step = rgbImg.width*3;
  // img.encoding = "rgb8";
  // img.data = rgbImg.rawData;

  // img.header.stamp = ros::Time::now();
  // img.header.frame_id = "FPV_CAMERA";
  // fpv_camera_stream_publisher_.publish(img);
}


void PSDKNode::publishMainCameraImage(CameraRGBImage rgbImg, void* userData)
{
  ROS_INFO("MAIN CAMERA CALLBACK");

  // sensor_msgs::Image img;
  // img.height = rgbImg.height;
  // img.width = rgbImg.width;
  // img.step = rgbImg.width*3;
  // img.encoding = "rgb8";
  // img.data = rgbImg.rawData;

  // img.header.stamp = ros::Time::now();
  // img.header.frame_id = "MAIN_CAMERA";
  // main_camera_stream_publisher_.publish(img);
}

static T_DjiReturnCode DjiUser_GetCurrentFileDirPath(const char *filePath, uint32_t pathBufferSize, char *dirPath)
{
    uint32_t i = strlen(filePath) - 1;
    uint32_t dirPathLen;

    while (filePath[i] != '/') {
        i--;
    }

    dirPathLen = i + 1;

    if (dirPathLen + 1 > pathBufferSize) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    memcpy(dirPath, filePath, dirPathLen);
    dirPath[dirPathLen] = 0;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

bool setupCameraStreamCallback(dji_osdk_ros::SetupCameraStream::Request& request,
                                         dji_osdk_ros::SetupCameraStream::Response& response)
{
  T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

  char fpvName[] = "FPV_CAM";
  char mainName[] = "MAIN_CAM";
  char viceName[] = "VICE_CAM";
  char topName[] = "TOP_CAM";

  auto *liveviewSample = new LiveviewSample();
  
  ROS_INFO_STREAM("called camera Stream Callback");

  if(request.cameraType == request.FPV_CAM)
  {
    if(request.start == 1)
    {
      char name[] = "FPV_CAM";
      response.result = liveviewSample->StartFpvCameraStream(&DjiUser_ShowRgbImageCallback, &fpvName);
    }
    else
    {
      response.result = liveviewSample->StopFpvCameraStream();
    }
  }
  else if(request.cameraType == request.MAIN_CAM)
  {
    if(request.start == 1)
    {
      response.result = liveviewSample->StartMainCameraStream(&DjiUser_ShowRgbImageCallback, &mainName);
    }
    else
    {
      response.result = liveviewSample->StopMainCameraStream();
    }
  }

  char isQuit;
  while (true) {
    cin >> isQuit;
    if (isQuit == 'q' || isQuit == 'Q') {
        break;
    }
  }

  osalHandler->TaskSleepMs(2000);

  return response.result;
}

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "psdk_node");
//   // PSDKNode psdk_node;
//   Application application;

//   ros::NodeHandle nh;

//   ros::ServiceServer setup_camera_stream_server_ = nh.advertiseService("setup_camera_stream", setupCameraStreamCallback);


//   ros::spin();
//   return 0;
// }

static int32_t s_demoIndex = -1;
char curFileDirPath[DJI_FILE_PATH_SIZE_MAX];

void DjiUser_RunCameraStreamViewSample()
{
    char cameraIndexChar = 0;
    char demoIndexChar = 0;
    char isQuit = 0;
    CameraRGBImage camImg;
    char fpvName[] = "FPV_CAM";
    char mainName[] = "MAIN_CAM";
    char viceName[] = "VICE_CAM";
    char topName[] = "TOP_CAM";
    auto *liveviewSample = new LiveviewSample();
    T_DjiReturnCode returnCode;

    returnCode = DjiUser_GetCurrentFileDirPath(__FILE__, DJI_FILE_PATH_SIZE_MAX, curFileDirPath);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        ROS_ERROR("Get file current path error, stat = 0x%08llX", returnCode);
    }

    cout << "Please choose the stream demo you want to run\n\n"
         << "--> [0] Normal RGB image display\n"
         << "--> [1] Binary image display\n"
         << "--> [2] Faces detection demo\n"
         << "--> [3] Tensorflow Object detection demo\n"
         << endl;
    cin >> demoIndexChar;

    switch (demoIndexChar) {
        case '0':
            s_demoIndex = 0;
            break;
        case '1':
            s_demoIndex = 1;
            break;
        case '2':
            s_demoIndex = 2;
            break;
        case '3':
            s_demoIndex = 3;
            break;
        default:
            cout << "No demo selected";
            delete liveviewSample;
            return;
    }

    cout << "Please enter the type of camera stream you want to view\n\n"
         << "--> [0] Fpv Camera\n"
         << "--> [1] Main Camera\n"
         << "--> [2] Vice Camera\n"
         << "--> [3] Top Camera\n"
         << endl;
    cin >> cameraIndexChar;

    switch (cameraIndexChar) {
        case '0':
            liveviewSample->StartFpvCameraStream(&DjiUser_ShowRgbImageCallback, &fpvName);
            break;
        case '1':
            liveviewSample->StartMainCameraStream(&DjiUser_ShowRgbImageCallback, &mainName);
            break;
        case '2':
            liveviewSample->StartViceCameraStream(&DjiUser_ShowRgbImageCallback, &viceName);
            break;
        case '3':
            liveviewSample->StartTopCameraStream(&DjiUser_ShowRgbImageCallback, &topName);
            break;
        default:
            cout << "No camera selected";
            delete liveviewSample;
            return;
    }

    cout << "Please enter the 'q' or 'Q' to quit camera stream view\n"
         << endl;

    while (true) {
        cin >> isQuit;
        if (isQuit == 'q' || isQuit == 'Q') {
            break;
        }
    }

    switch (cameraIndexChar) {
        case '0':
            liveviewSample->StopFpvCameraStream();
            break;
        case '1':
            liveviewSample->StopMainCameraStream();
            break;
        case '2':
            liveviewSample->StopViceCameraStream();
            break;
        case '3':
            liveviewSample->StopTopCameraStream();
            break;
        default:
            cout << "No camera selected";
            delete liveviewSample;
            return;
    }

    delete liveviewSample;
}


int main(int argc, char **argv)
{
    Application application();
    char inputChar;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

start:
    std::cout
        << "\n"
        << "| Available commands:                                                                              |\n"
        << "| [0] Fc subscribe sample - subscribe quaternion and gps data                                      |\n"
        << "| [1] Flight controller sample - take off landing                                                  |\n"
        << "| [2] Flight controller sample - take off position ctrl landing                                    |\n"
        << "| [3] Flight controller sample - take off go home force landing                                    |\n"
        << "| [4] Flight controller sample - take off velocity ctrl landing                                    |\n"
        << "| [5] Flight controller sample - arrest flying                                                     |\n"
        << "| [6] Flight controller sample - set get parameters                                                |\n"
        << "| [7] Hms info sample - get health manger system info                                              |\n"
        << "| [8] Waypoint 2.0 sample - run airline mission by settings (only support on M300 RTK)             |\n"
        << "| [9] Waypoint 3.0 sample - run airline mission by kmz file (not support on M300 RTK)              |\n"
        << "| [a] Gimbal manager sample                                                                        |\n"
        << "| [c] Camera stream view sample - display the camera video stream                                  |\n"
        << "| [d] Stereo vision view sample - display the stereo image                                         |\n"
        << "| [e] Start camera all features sample - you can operate the camera on DJI Pilot                   |\n"
        << "| [f] Start gimbal all features sample - you can operate the gimbal on DJI Pilot                   |\n"
        << "| [g] Start widget all features sample - you can operate the widget on DJI Pilot                   |\n"
        << "| [h] Start widget speaker sample - you can operate the speaker on DJI Pilot2                      |\n"
        << "| [i] Start power management sample - you will see notification when aircraft power off            |\n"
        << "| [j] Start data transmission sample - you can send or recv custom data on MSDK demo               |\n"
        << "| [k] Run camera manager sample - you can test camera's functions interactively                    |\n"
        << std::endl;

    std::cin >> inputChar;
    switch (inputChar) {
        // case '0':
        //     DjiTest_FcSubscriptionRunSample();
        //     break;
        // case '1':
        //     DjiTest_FlightControlRunSample(E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_LANDING);
        //     break;
        // case '2':
        //     DjiTest_FlightControlRunSample(E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_POSITION_CTRL_LANDING);
        //     break;
        // case '3':
        //     DjiTest_FlightControlRunSample(E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_GO_HOME_FORCE_LANDING);
        //     break;
        // case '4':
        //     DjiTest_FlightControlRunSample(E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_VELOCITY_CTRL_LANDING);
        //     break;
        // case '5':
        //     DjiTest_FlightControlRunSample(E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_ARREST_FLYING);
        //     break;
        // case '6':
        //     DjiTest_FlightControlRunSample(E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_SET_GET_PARAM);
        //     break;
        // case '7':
        //     DjiTest_HmsRunSample();
        //     break;
        // case '8':
        //     DjiTest_WaypointV2RunSample();
        //     break;
        // case '9':
        //     DjiTest_WaypointV3RunSample();
        //     break;
        // case 'a':
        //     DjiUser_RunGimbalManagerSample();
        //     break;
        case 'c':
            DjiUser_RunCameraStreamViewSample();
            break;
        // case 'd':
        //     DjiUser_RunStereoVisionViewSample();
        //     break;
        // case 'e':
        //     returnCode = DjiTest_CameraEmuBaseStartService();
        //     if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        //         ROS_ERROR("camera emu common init error");
        //         break;
        //     }

        //     if (DjiPlatform_GetSocketHandler() != nullptr) {
        //         returnCode = DjiTest_CameraEmuMediaStartService();
        //         if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        //             ROS_ERROR("camera emu media init error");
        //             break;
        //         }
        //     }

        //     USER_LOG_INFO("Start camera all feautes sample successfully");
        //     break;
        // case 'f':
        //     if (DjiTest_GimbalStartService() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        //         ROS_ERROR("psdk gimbal init error");
        //         break;
        //     }

        //     USER_LOG_INFO("Start gimbal all feautes sample successfully");
        //     break;
        // case 'g':
        //     returnCode = DjiTest_WidgetStartService();
        //     if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        //         ROS_ERROR("widget sample init error");
        //         break;
        //     }

        //     USER_LOG_INFO("Start widget all feautes sample successfully");
        //     break;
        // case 'h':
        //     returnCode = DjiTest_WidgetSpeakerStartService();
        //     if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        //         ROS_ERROR("widget speaker test init error");
        //         break;
        //     }

        //     USER_LOG_INFO("Start widget speaker sample successfully");
        //     break;
        // case 'i':
        //     applyHighPowerHandler.pinInit = DjiTest_HighPowerApplyPinInit;
        //     applyHighPowerHandler.pinWrite = DjiTest_WriteHighPowerApplyPin;

        //     returnCode = DjiTest_RegApplyHighPowerHandler(&applyHighPowerHandler);
        //     if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        //         ROS_ERROR("regsiter apply high power handler error");
        //         break;
        //     }

        //     returnCode = DjiTest_PowerManagementStartService();
        //     if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        //         ROS_ERROR("power management init error");
        //         break;
        //     }

        //     USER_LOG_INFO("Start power management sample successfully");
        //     break;
        // case 'j':
        //     returnCode = DjiTest_DataTransmissionStartService();
        //     if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        //         ROS_ERROR("data transmission sample init error");
        //         break;
        //     }

        //     USER_LOG_INFO("Start data transmission sample successfully");
        //     break;
        // case 'k':
        //     DjiUser_RunCameraManagerSample();
        //     break;
        default:
            break;
    }

    osalHandler->TaskSleepMs(2000);

    goto start;
}
