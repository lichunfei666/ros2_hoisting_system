/*
 * Copyright (c) 2024, DJI. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef DJI_PSDK_WRAPPER_PSDK_WRAPPER_HPP
#define DJI_PSDK_WRAPPER_PSDK_WRAPPER_HPP

#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <functional>
#include <atomic>
#include "dji_psdk_wrapper/config_manager.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// PSDK头文件
#ifdef __cplusplus
extern "C" {
#endif
#include "dji_typedef.h"
#include "dji_platform.h"
#include "dji_gimbal.h"
#include "dji_fc_subscription.h"
#include "dji_waypoint_v3.h"

#ifdef __cplusplus
}
#endif

// GPS数据类型定义
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GPS定位类型枚举
 */
typedef enum {
    DJI_GPS_POSITION_TYPE_NO_FIX = 0, /*!< 未定位 */
    DJI_GPS_POSITION_TYPE_SINGLE = 1, /*!< 单点定位 */
    DJI_GPS_POSITION_TYPE_FIX = 2, /*!< 固定解 */
    DJI_GPS_POSITION_TYPE_FLOAT = 3, /*!< 浮点解 */
    DJI_GPS_POSITION_TYPE_SIMULATED = 4, /*!< 模拟数据 */
} E_DjiGpsPositionType;

/**
 * @brief GPS数据结构体
 */
typedef struct {
    dji_f64_t latitude; /*!< 纬度，单位：度 */
    dji_f64_t longitude; /*!< 经度，单位：度 */
    dji_f64_t altitude; /*!< 高度，单位：米 */
    dji_f32_t positionStandardDeviation[3]; /*!< 位置标准差，单位：米 */
    E_DjiGpsPositionType positionType; /*!< 定位类型 */
    uint8_t satelliteCount; /*!< 卫星数量 */
    uint32_t timestampMs; /*!< 时间戳，单位：毫秒 */
    uint32_t timestampUs; /*!< 时间戳，单位：微秒 */
} T_DjiGpsData;

/**
 * @brief IMU数据结构体
 */
typedef struct {
    dji_f32_t quaternion[4]; /*!< 四元数，单位：无 */
    dji_f32_t gyroscope[3]; /*!< 陀螺仪数据，单位：rad/s */
    dji_f32_t accelerometer[3]; /*!< 加速度计数据，单位：m/s² */
    uint32_t timestampMs; /*!< 时间戳，单位：毫秒 */
    uint32_t timestampUs; /*!< 时间戳，单位：微秒 */
} T_DjiImuData;

/**
 * @brief 飞行模式枚举
 */
typedef enum {
    DJI_FLIGHT_MODE_MANUAL = 0, /*!< 手动模式 */
    DJI_FLIGHT_MODE_ATTITUDE = 1, /*!< 姿态模式 */
    DJI_FLIGHT_MODE_GPS = 2, /*!< GPS模式 */
    DJI_FLIGHT_MODE_ATTI_GPS = 3, /*!< 姿态GPS模式 */
    DJI_FLIGHT_MODE_SPORT = 4, /*!< 运动模式 */
    DJI_FLIGHT_MODE_GOPRO_FPV = 5, /*!< GoPro FPV模式 */
    DJI_FLIGHT_MODE_MISSION = 6, /*!< 任务模式 */
    DJI_FLIGHT_MODE_HOTPOINT = 7, /*!< 热点环绕模式 */
    DJI_FLIGHT_MODE_FOLLOW_ME = 8, /*!< 跟随模式 */
    DJI_FLIGHT_MODE_WAYPOINT = 9, /*!< 航点模式 */
    DJI_FLIGHT_MODE_CALIBRATION = 10, /*!< 校准模式 */
    DJI_FLIGHT_MODE_OTHER = 11, /*!< 其他模式 */
} T_DjiFlightMode;

/**
 * @brief 电池信息结构体
 */
typedef struct {
    uint8_t batteryId; /*!< 电池ID */
    double voltage; /*!< 电池电压，单位：V */
    double current; /*!< 电池电流，单位：A */
    int8_t percentage; /*!< 电池电量百分比，范围：0-100 */
    double temperature; /*!< 电池温度，单位：摄氏度 */
    uint32_t timestampMs; /*!< 时间戳，单位：毫秒 */
    uint32_t timestampUs; /*!< 时间戳，单位：微秒 */
} T_DjiBatteryInfo;

/**
 * @brief 多电池信息结构体
 */
typedef struct {
    T_DjiBatteryInfo battery1; /*!< 第一块电池信息 */
    T_DjiBatteryInfo battery2; /*!< 第二块电池信息 */
    bool battery1Available; /*!< 第一块电池信息是否可用 */
    bool battery2Available; /*!< 第二块电池信息是否可用 */
} T_DjiMultiBatteryInfo;


/**
 * @brief 无人机状态结构体
 */
typedef struct {
    uint8_t flightStatus; /*!< 飞行状态 */
    T_DjiFlightMode flightMode; /*!< 飞行模式 */
    uint8_t batteryPercentage; /*!< 电池电量百分比 */
    uint8_t gpsSignalLevel; /*!< GPS信号强度 */
    uint8_t satelliteCount; /*!< 卫星数量 */
    bool isMotorOn; /*!< 电机是否开启 */
    bool isFlying; /*!< 是否正在飞行 */
    bool isGpsFixed; /*!< GPS是否已定位 */
} T_DjiVehicleState;

#ifdef __cplusplus
}
#endif

namespace dji_psdk_wrapper
{



/**
 * @brief PSDK包装器类
 * 
 * 封装了DJI PSDK的核心功能，提供了与ROS 2集成的接口
 * 负责PSDK的初始化、启动、停止和重置
 * 提供无人机状态查询、飞行控制、任务管理等功能
 */
class PSDKWrapper
{
public:
  /**
   * @brief 构造函数
   */
  PSDKWrapper();
  
  /**
   * @brief 析构函数
   */
  ~PSDKWrapper();

  /**
   * @brief 初始化PSDK
   * @return bool 初始化是否成功
   */
  bool initialize();
  
  /**
   * @brief 启动PSDK
   * @return bool 启动是否成功
   */
  bool start();
  
  /**
   * @brief 停止PSDK
   */
  void stop();
  
  /**
   * @brief 重置PSDK
   */
  void reset();
  
  /**
   * @brief 检查PSDK是否已初始化
   * @return bool 是否已初始化
   */
  bool isInitialized() const;
  
  /**
   * @brief 检查PSDK是否正在运行
   * @return bool 是否正在运行
   */
  bool isRunning() const;
  
  /**
   * @brief 获取最后一次错误码
   * @return T_DjiReturnCode 错误码
   */
  T_DjiReturnCode getLastError() const;
  
  /**
   * @brief 获取当前连接状态
   * @return bool 连接状态，true表示已连接，false表示未连接
   */
  bool getConnectionStatus() const;
  
  /**
   * @brief 设置线程配置
   * @param priority 线程优先级
   * @param affinity 线程亲和性
   */
  void setThreadConfig(int priority, int affinity);
  
  /**
   * @brief 设置波特率
   * @param baudRate 波特率
   */
  void setBaudRate(uint32_t baudRate);
  
  /**
   * @brief 获取无人机状态
   * @param[out] state 无人机状态指针
   * @return T_DjiReturnCode 执行结果
   */
  T_DjiReturnCode getDroneState(T_DjiVehicleState *state);
  
  /**
   * @brief 获取GPS数据
   * @param[out] gpsData GPS数据指针
   * @return T_DjiReturnCode 执行结果
   */
  T_DjiReturnCode getGpsData(T_DjiGpsData *gpsData);

  /**
   * @brief 启动电机
   * @return T_DjiReturnCode 执行结果
   */
  T_DjiReturnCode turnOnMotors();
  
  /**
   * @brief 关闭电机
   * @return T_DjiReturnCode 执行结果
   */
  T_DjiReturnCode turnOffMotors();
  
  /**
   * @brief 紧急停止
   * @return T_DjiReturnCode 执行结果
   */
  T_DjiReturnCode emergencyStop();
  
  /**
   * @brief 获取IMU数据
   * @param[out] imuData IMU数据指针
   * @return T_DjiReturnCode 执行结果
   */
  T_DjiReturnCode getImuData(T_DjiImuData *imuData);

  /**
   * @brief 获取电池信息
   * @param[out] batteryInfo 电池信息指针
   * @return T_DjiReturnCode 执行结果
   */
  T_DjiReturnCode getBatteryInfo(T_DjiBatteryInfo *batteryInfo);

  /**
   * @brief 获取多电池信息（同时获取两块单体电池的信息）
   * @param[out] multiBatteryInfo 多电池信息指针
   * @return T_DjiReturnCode 执行结果
   */
  T_DjiReturnCode getMultiBatteryInfo(T_DjiMultiBatteryInfo *multiBatteryInfo);

  /**
   * @brief 配置PSDK
   * @return bool 配置是否成功
   */
  bool configurePsdk();
  
  /**
   * @brief 初始化平台
   * @return bool 初始化是否成功
   */
  bool initializePlatform();
  
  /**
   * @brief 初始化PSDK核心
   * @return bool 初始化是否成功
   */
  bool initializePsdkCore();
  
  /**
   * @brief 更新飞行控制配置
   * @param configPath 配置文件路径
   * @return T_DjiReturnCode 执行结果
   */
  T_DjiReturnCode updateFlightControlConfig(const std::string &configPath);
  
  /**
   * @brief 初始化车辆
   * @return bool 初始化是否成功
   */
  bool initializeVehicle();
  
  /**
   * @brief 注册回调函数
   * @return bool 注册是否成功
   */
  bool registerCallbacks();

  /**
   * @brief 注册状态回调函数
   * @param[in] callback 状态回调函数
   */
  void registerStateCallback(std::function<void(const T_DjiVehicleState &)> callback);
  
  /**
   * @brief 注册GPS回调函数
   * @param[in] callback GPS回调函数
   */
  void registerGpsCallback(std::function<void(const T_DjiGpsData &)> callback);
  
  /**
   * @brief 注册IMU回调函数
   * @param[in] callback IMU回调函数
   */
  void registerImuCallback(std::function<void(const T_DjiImuData &)> callback);
  
  /**
   * @brief 初始化Waypoint V3模块
   * @return T_DjiReturnCode 执行结果
   */
  T_DjiReturnCode initializeWaypointV3();
  
  /**
   * @brief 上传Waypoint V3航线文件
   * @param filePath 航线文件路径
   * @return T_DjiReturnCode 执行结果
   */
  T_DjiReturnCode uploadWaypointV3File(const std::string &filePath);
  
  /**
   * @brief 执行Waypoint V3任务操作
   * @param action 任务操作类型
   * @return T_DjiReturnCode 执行结果
   */
  T_DjiReturnCode executeWaypointV3Action(E_DjiWaypointV3Action action);
  
  /**
   * @brief 等待飞行状态变化
   * @param status 目标飞行状态
   * @return T_DjiReturnCode 执行结果
   */
  T_DjiReturnCode waitEndFlightStatus(T_DjiFcSubscriptionFlightStatus status);


private:
  // PSDK相关句柄
  T_DjiOsalHandler osalHandler_;      // 操作系统抽象层句柄
  

  
  // 线程相关
  std::unique_ptr<std::thread> psdkThread_;  // PSDK线程
  std::mutex psdkMutex_;            // PSDK互斥锁
  std::mutex configMutex_;          // 配置互斥锁
  std::atomic<bool> running_;       // 运行状态标志
  std::atomic<bool> initialized_;   // 初始化状态标志
  std::atomic<bool> isConnected_;   // 连接状态标志
  int connectionFailureCount_;      // 连接失败计数器
  int maxConnectionFailures_;       // 最大连接失败次数
  
  // 线程配置
  int threadPriority_;              // 线程优先级
  int threadAffinity_;              // 线程亲和性
  
  // 波特率配置
  uint32_t baudRate_;               // 串口波特率
  
  // 错误管理
  std::atomic<T_DjiReturnCode> lastError_;  // 最后一次错误码
  
  // 回调函数
  std::function<void(const T_DjiVehicleState &)> stateCallback_;  // 状态回调
  std::function<void(const T_DjiGpsData &)> gpsCallback_;  // GPS回调
  std::function<void(const T_DjiImuData &)> imuCallback_;  // IMU回调
  std::mutex callbackMutex_;  // 回调函数互斥锁
  
  // 更新频率配置（毫秒）
  int vehicleStateFreq_;  // 车辆状态更新频率
  int gpsDataFreq_;       // GPS数据更新频率
  int imuDataFreq_;       // IMU数据更新频率
  int batteryInfoFreq_;   // 电池信息更新频率
  
  // 数据更新检测相关
  std::chrono::steady_clock::time_point lastDataTime_;  // 最近一次数据更新时间
  

  
  /**
   * @brief PSDK线程主函数
   */
  void psdkThreadFunc();
  
  /**
   * @brief 处理PSDK事件
   */
  void handlePsdkEvents();
  
  /**
   * @brief 更新系统状态
   */
  void updateSystemState();
  
  /**
   * @brief 检查状态变化
   * @param currentState 当前状态
   */
  void checkStateChanges(const T_DjiVehicleState& currentState);
  
  /**
   * @brief 处理错误
   * @param errorCode 错误码
   * @param message 错误消息
   */
  void handleError(T_DjiReturnCode errorCode, const std::string &message);
  
  /**
   * @brief 获取错误描述
   * @param errorCode 错误码
   * @return std::string 错误描述
   */
  std::string getErrorDescription(T_DjiReturnCode errorCode);
  
  /**
   * @brief 设置线程优先级
   * @param handle 线程句柄
   * @param priority 优先级
   * @return bool 设置是否成功
   */
  bool setThreadPriority(std::thread::native_handle_type handle, int priority);
  
  /**
   * @brief 设置线程亲和性
   * @param handle 线程句柄
   * @param coreId 核心ID
   * @return bool 设置是否成功
   */
  bool setThreadAffinity(std::thread::native_handle_type handle, int coreId);
  

  
};

}  // namespace dji_psdk_wrapper

#endif  // DJI_PSDK_WRAPPER_PSDK_WRAPPER_HPP