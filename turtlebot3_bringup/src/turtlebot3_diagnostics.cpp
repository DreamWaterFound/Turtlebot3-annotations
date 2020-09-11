/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
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

/* Authors: Taehoon Lim (Darby) */

// 这个程序用于对 turtlebot3 机器人的工作状态进行诊断


// ==============================================  头文件  ========================================
// ROS
#include <ros/ros.h>                                // ROS 支持
#include <sensor_msgs/BatteryState.h>               // 电池状态类型
#include <sensor_msgs/Imu.h>                        // IMU 信息类型
#include <sensor_msgs/MagneticField.h>              // 磁场信息类型
#include <sensor_msgs/LaserScan.h>                  // 激光雷达信息类型
#include <diagnostic_msgs/DiagnosticArray.h>        // 诊断消息类型
#include <turtlebot3_msgs/SensorState.h>            // 传感器状态数据类型
#include <turtlebot3_msgs/VersionInfo.h>            // 版本信息
// STL
#include <string>

// ==============================================  宏定义  ========================================
// 各种版本信息
#define SOFTWARE_VERSION "1.2.3"
#define HARDWARE_VERSION "2020.03.16"
#define FIRMWARE_VERSION_MAJOR_NUMBER 1
#define FIRMWARE_VERSION_MINOR_NUMBER 2

// ==============================================  全局变量  ========================================
ros::Publisher tb3_version_info_pub;                // 版本信息发布器
ros::Publisher tb3_diagnostics_pub;                 // 诊断信息发布器

diagnostic_msgs::DiagnosticStatus imu_state;        // IMU 的诊断信息
diagnostic_msgs::DiagnosticStatus motor_state;      // 电机 的诊断信息
diagnostic_msgs::DiagnosticStatus LDS_state;        // 激光雷达 的诊断信息
diagnostic_msgs::DiagnosticStatus battery_state;    // 电池 的诊断信息
diagnostic_msgs::DiagnosticStatus button_state;     // 用户按钮 的诊断信息

// ==============================================  前视声明  ========================================
// 版本信息的结构体
typedef struct
{
  int major_number;
  int minor_number;
  int patch_number;
}VERSION;

/**
 * @brief 用于分隔版本字符串到子字符串中
 * @param[in]  data       // 等待处理的字符串
 * @param[in]  separator  // 分割符
 * @param[out] temp       // 输出
 */
void split(std::string data, std::string separator, std::string* temp)
{
  // 找到的内容计数，用于指定输出 temp 的下标
	int cnt = 0;
  std::string copy = data;
  
  // 准备循环
	while(true)
	{
    // 找到第一个分隔符出现的位置
		std::size_t index = copy.find(separator);

    // 如果找到了
    if (index != std::string::npos)
    {
      // 分割数据并保存
      temp[cnt] = copy.substr(0, index);
      // 准备分割后的原字符串进行下一次循环
      copy = copy.substr(index+1, copy.length());
    }
    else
    {
      // 说明没有了，将剩下的数据全部保存到最后的 temp 数组元素里面
      temp[cnt] = copy.substr(0, copy.length());
      break;
    }
    // 输出 temp 输出计数
		++cnt;
	}
}

/**
 * @brief 设置诊断信息
 * @param[in] diag          要操作的诊断消息对象
 * @param[in] level         错误等级
 * @param[in] name          传感器类型
 * @param[in] message       对传感器工作状态的文字描述
 * @param[in] hardware_id   硬件标识
 */
void setDiagnosisMsg(diagnostic_msgs::DiagnosticStatus *diag, uint8_t level, std::string name, std::string message, std::string hardware_id)
{
  diag->level       = level;
  diag->name        = name;
  diag->message     = message;
  diag->hardware_id = hardware_id;
}

/** @brief 设置 IMU 诊断信息
 *  @param[in] level         错误等级
 *  @param[in] message       文字描述 */
void setIMUDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&imu_state, level, "IMU Sensor", message, "MPU9250");
}

/** @brief 设置 电机 诊断信息
 *  @param[in] level         错误等级
 *  @param[in] message       文字描述 */
void setMotorDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&motor_state, level, "Actuator", message, "DYNAMIXEL X");
}

/** @brief 设置 电池 诊断信息
 *  @param[in] level         错误等级
 *  @param[in] message       文字描述 */
void setBatteryDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&battery_state, level, "Power System", message, "Battery");
}

/** @brief 设置 激光雷达 诊断信息
 *  @param[in] level         错误等级
 *  @param[in] message       文字描述 */
void setLDSDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&LDS_state, level, "Lidar Sensor", message, "HLS-LFCD-LDS");
}

/** @brief 设置 用户按钮 诊断信息
 *  @param[in] level         错误等级
 *  @param[in] message       文字描述 */
void setButtonDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&button_state, level, "Analog Button", message, "OpenCR Button");
}

// ----

// IMU 消息的回调函数
void imuMsgCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  setIMUDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}

// 雷达消息的回调函数
void LDSMsgCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  setLDSDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}

// 综合楼多种传感器消息的回调函数
void sensorStateMsgCallback(const turtlebot3_msgs::SensorState::ConstPtr &msg)
{
  // Step 1 判断电量
  if (msg->battery > 11.0)
    // 正常
    setBatteryDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
  else
    // 需要充电
    setBatteryDiagnosis(diagnostic_msgs::DiagnosticStatus::WARN, "Charge!!! Charge!!!");

  // Step 2 检查用户按键
  if (msg->button == turtlebot3_msgs::SensorState::BUTTON0)
    setButtonDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "BUTTON 0 IS PUSHED");
  else if (msg->button == turtlebot3_msgs::SensorState::BUTTON1)
    setButtonDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "BUTTON 1 IS PUSHED");
  else
    setButtonDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Pushed Nothing");

  // Step 3 是否打开力矩控制
  if (msg->torque == true)
    setMotorDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Torque ON");
  else
    setMotorDiagnosis(diagnostic_msgs::DiagnosticStatus::WARN, "Torque OFF");
}

// 版本信息的回调函数
void firmwareVersionMsgCallback(const turtlebot3_msgs::VersionInfo::ConstPtr &msg)
{
  // Step 0 数据准备
  // 标记是否已经检测过版本
  static bool check_version = false;
  // 保存分割后的版本信息
  std::string get_version[3];

  // Step 1 分隔版本字符串
  split(msg->firmware, ".", get_version);

  // 版本信息转换成为数字
  VERSION firmware_version; 
  firmware_version.major_number = std::stoi(get_version[0]);
  firmware_version.minor_number = std::stoi(get_version[1]);
  firmware_version.patch_number = std::stoi(get_version[2]);

  // Step 2 如果还没有检测过版本
  if (check_version == false)
  {
    // 版本检测
    if (firmware_version.major_number == FIRMWARE_VERSION_MAJOR_NUMBER)
    {
      if (firmware_version.minor_number > FIRMWARE_VERSION_MINOR_NUMBER)
      {
        ROS_WARN("This firmware(v%s) isn't compatible with this software (v%s)", msg->firmware.data(), SOFTWARE_VERSION);
        ROS_WARN("You can find how to update its in `FAQ` section(turtlebot3.robotis.com)");
      }
    }
    else
    {
      ROS_WARN("This firmware(v%s) isn't compatible with this software (v%s)", msg->firmware.data(), SOFTWARE_VERSION);
      ROS_WARN("You can find how to update its in `FAQ` section(turtlebot3.robotis.com)");
    }
    // 标记已经检查过版本了
    check_version = true;
  }
  
  // Step 3 构造并且发布版本信息
  turtlebot3_msgs::VersionInfo version;

  version.software = SOFTWARE_VERSION;
  version.hardware = HARDWARE_VERSION;
  version.firmware = msg->firmware;

  tb3_version_info_pub.publish(version);
}

/** @brief 发布消息 */
void msgPub()
{
  // 构造诊断信息
  diagnostic_msgs::DiagnosticArray tb3_diagnostics;

  tb3_diagnostics.header.stamp = ros::Time::now();

  // 追加信息，就是读的时候也需要按照相应的顺序去读；不过有键值对的形式倒也不用担心太多
  tb3_diagnostics.status.clear();
  tb3_diagnostics.status.push_back(imu_state);
  tb3_diagnostics.status.push_back(motor_state);
  tb3_diagnostics.status.push_back(LDS_state);
  tb3_diagnostics.status.push_back(battery_state);
  tb3_diagnostics.status.push_back(button_state);

  tb3_diagnostics_pub.publish(tb3_diagnostics);
}

/**
 * @brief 主函数
 * @param[in] argc 参数个数
 * @param[in] argv 参数表列
 * @return int 运行返回值
 */
int main(int argc, char **argv)
{
  // Step 1 初始化当前节点
  ros::init(argc, argv, "turtlebot3_diagnostic");
  ros::NodeHandle nh;

  // Step 2 初始化诊断信息和版本信息的发布器
  tb3_diagnostics_pub  = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
  tb3_version_info_pub = nh.advertise<turtlebot3_msgs::VersionInfo>("version_info", 10);

  // Step 3 初始化相关传感器信息的订阅器并指定回调函数
  ros::Subscriber imu         = nh.subscribe("imu", 10, imuMsgCallback);
  ros::Subscriber lds         = nh.subscribe("scan", 10, LDSMsgCallback);
  // 综合了其他低速传感器信息
  ros::Subscriber tb3_sensor  = nh.subscribe("sensor_state", 10, sensorStateMsgCallback);
  // 版本信息
  ros::Subscriber version     = nh.subscribe("firmware_version", 10, firmwareVersionMsgCallback);

  // Step 4 进入 1Hz 的循环
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    // 循环发布消息
    msgPub();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
