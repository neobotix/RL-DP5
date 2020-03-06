#pragma once

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <cstdint>
#include <cstring>
#include <atomic>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <cassert>
#include <chrono>
#include <sstream>

#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointJog.h>
#define _USE_MATH_DEFINES
#include <cmath>

#include "cpr_robot/GetRobotInfo.h"
#include "cpr_robot/GetJointInfo.h"
#include "cpr_robot/RobotState.h"
#include "cpr_robot/ChannelStates.h"
#include "cpr_robot/RobotCommand.h"

#include "cpr_robot/Bus.h"
#include "cpr_robot/MotorModule.h"
#include "cpr_robot/Joint.h"
#include "cpr_robot/Robot.h"
#include "cpr_robot/igus_4DOF_SV.h"
#include "cpr_robot/igus_5DOF_SV.h"
#include "cpr_robot/CPRMover6.h"

//! \namespace cpr_robot Provides everything needed to control a robot over a CAN bus connection within a ROS environment.
