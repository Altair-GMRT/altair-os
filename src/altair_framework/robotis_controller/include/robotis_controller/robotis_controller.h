/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*
 * robotis_controller.h
 *
 *  Created on: 2016. 1. 15.
 *      Author: zerom
 */

#ifndef ROBOTIS_CONTROLLER_ROBOTIS_CONTROLLER_H_
#define ROBOTIS_CONTROLLER_ROBOTIS_CONTROLLER_H_


#include <ros/ros.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <pthread.h>

#include "robotis_controller_msgs/WriteControlTable.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/GetJointModule.h"
#include "robotis_controller_msgs/SetJointModule.h"
#include "robotis_controller_msgs/SetModule.h"

#include "robotis_device/robot.h"
#include "robotis_framework_common/motion_module.h"
#include "robotis_framework_common/sensor_module.h"
#include "dynamixel_sdk/group_bulk_read.h"
#include "dynamixel_sdk/group_sync_write.h"

namespace robotis_framework
{

enum ControllerMode
{
  MotionModuleMode,
  DirectControlMode
};

class RobotisController : public Singleton<RobotisController>
{
private:
  boost::thread   queue_thread_;
  boost::thread   gazebo_thread_;
  boost::thread   set_module_thread_;
  boost::mutex    queue_mutex_;

  bool            init_pose_loaded_;
  bool            is_timer_running_;
  bool            stop_timer_;
  pthread_t       timer_thread_;
  ControllerMode  controller_mode_;

  std::list<MotionModule *> motion_modules_;
  std::list<SensorModule *> sensor_modules_;
  std::vector<dynamixel::GroupSyncWrite *>  direct_sync_write_;

  std::map<std::string, double> sensor_result_;

  void gazeboTimerThread();
  void msgQueueThread();
  void setCtrlModuleThread(std::string ctrl_module);
  void setJointCtrlModuleThread(const robotis_controller_msgs::JointCtrlModule::ConstPtr &msg);

  bool isTimerStopped();
  void initializeSyncWrite();

public:
  bool              DEBUG_PRINT;
  Robot            *robot_;

  bool              gazebo_mode_;
  std::string       gazebo_robot_name_;

  /* bulk read */
  std::map<std::string, dynamixel::GroupBulkRead *>   port_to_bulk_read_;

  /* sync write */
  std::map<std::string, dynamixel::GroupSyncWrite *>  port_to_sync_write_position_;
  std::map<std::string, dynamixel::GroupSyncWrite *>  port_to_sync_write_velocity_;
  std::map<std::string, dynamixel::GroupSyncWrite *>  port_to_sync_write_current_;
  std::map<std::string, dynamixel::GroupSyncWrite *>  port_to_sync_write_position_p_gain_;
  std::map<std::string, dynamixel::GroupSyncWrite *>  port_to_sync_write_position_i_gain_;
  std::map<std::string, dynamixel::GroupSyncWrite *>  port_to_sync_write_position_d_gain_;
  std::map<std::string, dynamixel::GroupSyncWrite *>  port_to_sync_write_velocity_p_gain_;
  std::map<std::string, dynamixel::GroupSyncWrite *>  port_to_sync_write_velocity_i_gain_;
  std::map<std::string, dynamixel::GroupSyncWrite *>  port_to_sync_write_velocity_d_gain_;

  /* publisher */
  ros::Publisher  goal_joint_state_pub_;
  ros::Publisher  present_joint_state_pub_;
  ros::Publisher  current_module_pub_;

  std::map<std::string, ros::Publisher> gazebo_joint_position_pub_;
  std::map<std::string, ros::Publisher> gazebo_joint_velocity_pub_;
  std::map<std::string, ros::Publisher> gazebo_joint_effort_pub_;

  static void *timerThread(void *param);

  RobotisController();

  bool    initialize(const std::string robot_file_path, const std::string init_file_path);
  void    initializeDevice(const std::string init_file_path);
  void    process();

  void    addMotionModule(MotionModule *module);
  void    removeMotionModule(MotionModule *module);
  void    addSensorModule(SensorModule *module);
  void    removeSensorModule(SensorModule *module);

  void    startTimer();
  void    stopTimer();
  bool    isTimerRunning();

  void    setCtrlModule(std::string module_name);
  void    loadOffset(const std::string path);

  /* ROS Topic Callback Functions */
  void    writeControlTableCallback(const robotis_controller_msgs::WriteControlTable::ConstPtr &msg);
  void    syncWriteItemCallback(const robotis_controller_msgs::SyncWriteItem::ConstPtr &msg);
  void    setControllerModeCallback(const std_msgs::String::ConstPtr &msg);
  void    setJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void    setJointCtrlModuleCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr &msg);
  void    setCtrlModuleCallback(const std_msgs::String::ConstPtr &msg);
  bool    getJointCtrlModuleService(robotis_controller_msgs::GetJointModule::Request &req, robotis_controller_msgs::GetJointModule::Response &res);
  bool    setJointCtrlModuleService(robotis_controller_msgs::SetJointModule::Request &req, robotis_controller_msgs::SetJointModule::Response &res);
  bool    setCtrlModuleService(robotis_controller_msgs::SetModule::Request &req, robotis_controller_msgs::SetModule::Response &res);

  void    gazeboJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);

  int     ping        (const std::string joint_name, uint8_t *error = 0);
  int     ping        (const std::string joint_name, uint16_t* model_number, uint8_t *error = 0);

  int     action      (const std::string joint_name);
  int     reboot      (const std::string joint_name, uint8_t *error = 0);
  int     factoryReset(const std::string joint_name, uint8_t option = 0, uint8_t *error = 0);

  int     read        (const std::string joint_name, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0);
  int     readCtrlItem(const std::string joint_name, const std::string item_name, uint32_t *data, uint8_t *error = 0);

  int     read1Byte   (const std::string joint_name, uint16_t address, uint8_t *data, uint8_t *error = 0);
  int     read2Byte   (const std::string joint_name, uint16_t address, uint16_t *data, uint8_t *error = 0);
  int     read4Byte   (const std::string joint_name, uint16_t address, uint32_t *data, uint8_t *error = 0);

  int     write       (const std::string joint_name, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0);
  int     writeCtrlItem(const std::string joint_name, const std::string item_name, uint32_t data, uint8_t *error = 0);

  int     write1Byte  (const std::string joint_name, uint16_t address, uint8_t data, uint8_t *error = 0);
  int     write2Byte  (const std::string joint_name, uint16_t address, uint16_t data, uint8_t *error = 0);
  int     write4Byte  (const std::string joint_name, uint16_t address, uint32_t data, uint8_t *error = 0);

  int     regWrite    (const std::string joint_name, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0);
};

}


#endif /* ROBOTIS_CONTROLLER_ROBOTIS_CONTROLLER_H_ */
