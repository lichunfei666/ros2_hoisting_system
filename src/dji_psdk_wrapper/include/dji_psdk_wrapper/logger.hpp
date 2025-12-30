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

#ifndef DJI_PSDK_WRAPPER_LOGGER_HPP
#define DJI_PSDK_WRAPPER_LOGGER_HPP

#include <string>
#include <memory>
#include <mutex>
#include <sstream>
#include <chrono>
#include <iostream>
#include <fstream>
#include <cstdarg>
#include <sys/stat.h>
#include <string.h>
#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"

namespace dji_psdk_wrapper
{

/**
 * @brief 日志级别枚举
 */
enum class LogLevel
{
  DEBUG = 0,    // 调试信息
  INFO = 1,     // 一般信息
  WARNING = 2,  // 警告信息
  ERROR = 3,    // 错误信息
  FATAL = 4     // 致命错误
};

/**
 * @brief 日志记录器类
 * 
 * 基于ROS2内置日志系统实现的日志记录器
 */
class Logger
{
public:
  /**
   * @brief 获取日志记录器单例实例
   * @return Logger& 日志记录器实例
   */
  static Logger& getInstance()
  {
    static Logger instance;
    return instance;
  }

  /**
   * @brief 设置日志级别
   * @param level 日志级别
   */
  void setLogLevel(LogLevel level)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    logLevel_ = level;
    
    // 根据新的日志级别更新ROS2日志器级别
    rcutils_ret_t ret = RCUTILS_RET_OK;
    switch (logLevel_)
    {
      case LogLevel::DEBUG:
        ret = rcutils_logging_set_logger_level("psdk_wrapper", RCUTILS_LOG_SEVERITY_DEBUG);
        break;
      case LogLevel::INFO:
        ret = rcutils_logging_set_logger_level("psdk_wrapper", RCUTILS_LOG_SEVERITY_INFO);
        break;
      case LogLevel::WARNING:
        ret = rcutils_logging_set_logger_level("psdk_wrapper", RCUTILS_LOG_SEVERITY_WARN);
        break;
      case LogLevel::ERROR:
        ret = rcutils_logging_set_logger_level("psdk_wrapper", RCUTILS_LOG_SEVERITY_ERROR);
        break;
      case LogLevel::FATAL:
        ret = rcutils_logging_set_logger_level("psdk_wrapper", RCUTILS_LOG_SEVERITY_FATAL);
        break;
      default:
        ret = rcutils_logging_set_logger_level("psdk_wrapper", RCUTILS_LOG_SEVERITY_INFO);
        break;
    }
    
    // 忽略返回值（在简化版中不处理错误）
    (void)ret;
  }

  /**
   * @brief 设置日志文件路径
   * 
   * 设置日志文件路径，支持简化版调用
   * 
   * @param filePath 日志文件路径
   */
  void setLogFile(const std::string& filePath)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    try
    {
      // 关闭当前日志文件流（如果已打开）
      if (logFileStream_.is_open())
      {
        logFileStream_.close();
      }
      
      // 更新日志文件路径
      logFilePath_ = filePath;
      
      // 尝试打开日志文件，使用覆盖模式而不是追加模式
      logFileStream_.open(logFilePath_, std::ios::out);
      if (logFileStream_.is_open())
      {
        // 启用文件日志功能
        enableFileLog_ = true;
        logFileStream_ << "[INFO] Logger: Log file opened successfully: " << logFilePath_ << std::endl;
      }
      else
      {
        std::cerr << "Failed to open log file: " << logFilePath_ << std::endl;
        enableFileLog_ = false;
      }
      
      // 不再创建终端日志文件，避免与节点日志重复
    }
    catch (const std::exception& e)
    {
      std::cerr << "Failed to set log file: " << e.what() << std::endl;
      enableFileLog_ = false;
      enableTerminalLogFile_ = false;
      if (logFileStream_.is_open())
      {
        logFileStream_.close();
      }
      if (terminalLogStream_.is_open())
      {
        terminalLogStream_.close();
      }
    }
  }

  /**
   * @brief 从配置文件加载日志配置
   * @param config YAML配置节点
   */
  void loadConfigFromYaml(const YAML::Node& config)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    try
    {
      if (config["logger"])
      {
        const auto& loggerConfig = config["logger"];
        
        // 加载日志级别
        if (loggerConfig["level"])
        {
          std::string levelStr = loggerConfig["level"].as<std::string>();
          if (levelStr == "DEBUG") setLogLevel(LogLevel::DEBUG);
          else if (levelStr == "INFO") setLogLevel(LogLevel::INFO);
          else if (levelStr == "WARNING") setLogLevel(LogLevel::WARNING);
          else if (levelStr == "ERROR") setLogLevel(LogLevel::ERROR);
          else if (levelStr == "FATAL") setLogLevel(LogLevel::FATAL);
        }
        
        // 加载控制台输出配置
        if (loggerConfig["console_output"])
        {
          enableConsoleLog(loggerConfig["console_output"].as<bool>());
        }
        
        // 加载文件输出配置
        if (loggerConfig["file_output"] && loggerConfig["file_output"].as<bool>())
        {
          if (loggerConfig["log_file"])
          {
            std::string logFile = loggerConfig["log_file"].as<std::string>();
            
            setLogFile(logFile);
          }
        }
      }
    }
    catch (const std::exception& e)
    {
      std::cerr << "Failed to load logger config from YAML: " << e.what() << std::endl;
    }
  }

  // 简化版不支持日志滚动功能
  void enableLogRotation(bool enable, size_t maxFiles = 5)
  {
    // 该功能在简化版中不可用
    (void)enable;
    (void)maxFiles;
  }

  /**
   * @brief 获取当前日志文件路径
   * @return std::string 当前日志文件路径
   */
  std::string getLogFilePath() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return logFilePath_;
  }

  /**
   * @brief 获取日志统计信息
   * @return std::string 日志统计信息
   */
  std::string getLogStats() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::stringstream ss;
    
    ss << "Log Level: " << getLevelString(logLevel_) << "\n"
       << "Console Log: " << (enableConsoleLog_ ? "Enabled" : "Disabled") << "\n"
       << "File Log: " << (enableFileLog_ ? "Enabled" : "Disabled") << "\n"
       << "Current Log File: " << logFilePath_;
    
    return ss.str();
  }

  /**
   * @brief 启用或禁用控制台日志
   * @param enable 是否启用
   */
  void enableConsoleLog(bool enable)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    enableConsoleLog_ = enable;
  }

  // 简化版不支持异步日志模式
  void enableAsyncMode(bool enable, size_t queueSize = 8192)
  {
    // 该功能在简化版中不可用
    (void)enable;
    (void)queueSize;
  }

  // 简化版不提供spdlog日志器实例
  void* getSpdLogger()
  {
    return nullptr;
  }

  /**
   * @brief 记录调试日志
   * @param tag 日志标签
   * @param message 日志消息
   */
  void debug(const std::string& tag, const std::string& message)
  {
    log(LogLevel::DEBUG, tag, message);
  }

  /**
   * @brief 记录调试日志（支持格式化字符串）
   * @param tag 日志标签
   * @param format 格式化字符串
   * @param ... 可变参数
   */
  void debug(const std::string& tag, const char* format, ...)
  {
    va_list args;
    va_start(args, format);
    char buffer[2048];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    log(LogLevel::DEBUG, tag, buffer);
  }

  /**
   * @brief 记录信息日志
   * @param tag 日志标签
   * @param message 日志消息
   */
  void info(const std::string& tag, const std::string& message)
  {
    log(LogLevel::INFO, tag, message);
  }

  /**
   * @brief 记录信息日志（支持格式化字符串）
   * @param tag 日志标签
   * @param format 格式化字符串
   * @param ... 可变参数
   */
  void info(const std::string& tag, const char* format, ...)
  {
    va_list args;
    va_start(args, format);
    char buffer[2048];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    log(LogLevel::INFO, tag, buffer);
  }

  /**
   * @brief 记录警告日志
   * @param tag 日志标签
   * @param message 日志消息
   */
  void warning(const std::string& tag, const std::string& message)
  {
    log(LogLevel::WARNING, tag, message);
  }

  /**
   * @brief 记录警告日志（支持格式化字符串）
   * @param tag 日志标签
   * @param format 格式化字符串
   * @param ... 可变参数
   */
  void warning(const std::string& tag, const char* format, ...)
  {
    va_list args;
    va_start(args, format);
    char buffer[2048];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    log(LogLevel::WARNING, tag, buffer);
  }

  /**
   * @brief 记录错误日志
   * @param tag 日志标签
   * @param message 日志消息
   */
  void error(const std::string& tag, const std::string& message)
  {
    log(LogLevel::ERROR, tag, message);
  }

  /**
   * @brief 记录错误日志（支持格式化字符串）
   * @param tag 日志标签
   * @param format 格式化字符串
   * @param ... 可变参数
   */
  void error(const std::string& tag, const char* format, ...)
  {
    va_list args;
    va_start(args, format);
    char buffer[2048];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    log(LogLevel::ERROR, tag, buffer);
  }

  /**
   * @brief 记录致命错误日志
   * @param tag 日志标签
   * @param message 日志消息
   */
  void fatal(const std::string& tag, const std::string& message)
  {
    log(LogLevel::FATAL, tag, message);
  }

  /**
   * @brief 记录致命错误日志（支持格式化字符串）
   * @param tag 日志标签
   * @param format 格式化字符串
   * @param ... 可变参数
   */
  void fatal(const std::string& tag, const char* format, ...)
  {
    va_list args;
    va_start(args, format);
    char buffer[2048];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    log(LogLevel::FATAL, tag, buffer);
  }

  /**
   * @brief 刷新日志（将缓冲区内容写入文件）
   */
  void flush()
  {
    // ROS2日志系统不需要手动刷新
  }

  /**
   * @brief 设置刷新间隔
   * @param interval_ms 刷新间隔（毫秒）
   */
  void setFlushInterval(size_t interval_ms)
  {
    // 简化版不支持自动刷新间隔
    (void)interval_ms;
  }
  
  /**
   * @brief 获取当前时间字符串（辅助方法）
   * @return std::string 格式化的时间字符串
   */
  /**
   * @brief 获取当前时间字符串（用于文件名）
   * @return std::string 格式化的时间字符串，适合作为文件名
   */
  std::string getCurrentTimeStringForFilename() const
  {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    
    // 使用线程安全的时间转换函数
    std::tm now_tm;
    #ifdef _WIN32
      localtime_s(&now_tm, &now_time_t);
    #else
      localtime_r(&now_time_t, &now_tm);
    #endif
    
    // 格式化时间字符串 - 适合作为文件名（包含年份）
    char time_str[50];
    std::snprintf(time_str, sizeof(time_str), "%04d%02d%02d_%02d%02d%02d",
                now_tm.tm_year + 1900,
                now_tm.tm_mon + 1,
                now_tm.tm_mday,
                now_tm.tm_hour,
                now_tm.tm_min,
                now_tm.tm_sec);
    
    return std::string(time_str);
  }
  
  std::string getCurrentTimeString() const
  {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    // 使用线程安全的时间转换函数
    std::tm now_tm;
    #ifdef _WIN32
      localtime_s(&now_tm, &now_time_t);
    #else
      localtime_r(&now_time_t, &now_tm);
    #endif
    
    // 格式化时间字符串 - 北京时间格式（去掉年份）
    char time_str[50];
    std::snprintf(time_str, sizeof(time_str), "%02d-%02d %02d:%02d:%02d.%03d",
                now_tm.tm_mon + 1,
                now_tm.tm_mday,
                now_tm.tm_hour,
                now_tm.tm_min,
                now_tm.tm_sec,
                static_cast<int>(now_ms.count()));
    
    return std::string(time_str);
  }

private:
  Logger() : 
    logLevel_(LogLevel::DEBUG), 
    enableConsoleLog_(true), 
    enableFileLog_(false),
    enableTerminalLogFile_(false), // 禁用终端日志文件，避免与dji_psdk_node.log重复
    logFilePath_(),
    terminalLogFilePath_("terminal.log") // 先使用默认值
  {
    // 不再生成终端日志文件，避免与节点日志重复
    
    // 初始化ROS2日志器级别为DEBUG，确保DEBUG级别日志能够显示
    rcutils_ret_t ret = rcutils_logging_set_logger_level("psdk_wrapper", RCUTILS_LOG_SEVERITY_DEBUG);
    (void)ret; // 忽略返回值
    
    // 打开终端日志文件
    try {
      terminalLogStream_.open(terminalLogFilePath_, std::ios::out);
      if (terminalLogStream_.is_open()) {
        terminalLogStream_ << "[" << getCurrentTimeString() << "] [INFO] Logger initialized, terminal log file opened: " << terminalLogFilePath_ << std::endl;
        terminalLogStream_.flush();
      } else {
        std::cerr << "Failed to open terminal log file: " << terminalLogFilePath_ << std::endl;
        enableTerminalLogFile_ = false;
      }
    } catch (const std::exception& e) {
      std::cerr << "Exception when opening terminal log file: " << e.what() << std::endl;
      enableTerminalLogFile_ = false;
    }
  }

  ~Logger()
  {
    // ROS2日志系统不需要手动清理资源
    
    // 关闭日志文件流
    std::lock_guard<std::mutex> lock(mutex_);
    if (logFileStream_.is_open())
    {
      logFileStream_ << "[INFO] Logger: Log file closed" << std::endl;
      logFileStream_.close();
    }
    
    // 关闭终端日志文件流
    if (terminalLogStream_.is_open())
    {
      terminalLogStream_.close();
    }
  }

  /**
   * @brief 获取日志级别字符串
   * @param level 日志级别
   * @return std::string 日志级别字符串
   */
  std::string getLevelString(LogLevel level) const
  {
    switch (level)
    {
      case LogLevel::DEBUG:   return "DEBUG";
      case LogLevel::INFO:    return "INFO";
      case LogLevel::WARNING: return "WARNING";
      case LogLevel::ERROR:   return "ERROR";
      case LogLevel::FATAL:   return "FATAL";
      default:                return "UNKNOWN";
    }
  }

  /**
   * @brief 记录日志
   * @param level 日志级别
   * @param tag 日志标签
   * @param message 日志消息
   */
  void log(LogLevel level, const std::string& tag, const std::string& message)
  {
    std::string formatted_message = "[" + tag + "] " + message;
    
    if (enableConsoleLog_)
    {
      // 获取当前时间（北京时间）
      auto now = std::chrono::system_clock::now();
      auto now_time_t = std::chrono::system_clock::to_time_t(now);
      auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          now.time_since_epoch()) % 1000;
      
      // 使用线程安全的时间转换函数
      std::tm now_tm;
      #ifdef _WIN32
        localtime_s(&now_tm, &now_time_t);
      #else
        localtime_r(&now_time_t, &now_tm);
      #endif
      
      // 格式化时间字符串 - 北京时间格式（去掉年份）
      char time_str[50];
      std::snprintf(time_str, sizeof(time_str), "%02d-%02d %02d:%02d:%02d.%03d",
                  now_tm.tm_mon + 1,
                  now_tm.tm_mday,
                  now_tm.tm_hour,
                  now_tm.tm_min,
                  now_tm.tm_sec,
                  static_cast<int>(now_ms.count()));
      time_str[sizeof(time_str) - 1] = '\0';
      
      // 格式化日志级别
      std::string level_str = getLevelString(level);
      
      // 直接输出到控制台，使用彩色和格式
      switch (level)
      {
        case LogLevel::DEBUG:
          if (logLevel_ <= LogLevel::DEBUG)
            std::cout << "[DEBUG] [" << time_str << "] " << formatted_message << std::endl;
          break;
        case LogLevel::INFO:
          if (logLevel_ <= LogLevel::INFO)
            std::cout << "[INFO] [" << time_str << "] " << formatted_message << std::endl;
          break;
        case LogLevel::WARNING:
          if (logLevel_ <= LogLevel::WARNING)
            std::cout << "[WARN] [" << time_str << "] " << formatted_message << std::endl;
          break;
        case LogLevel::ERROR:
          if (logLevel_ <= LogLevel::ERROR)
            std::cerr << "[ERROR] [" << time_str << "] " << formatted_message << std::endl;
          break;
        case LogLevel::FATAL:
          if (logLevel_ <= LogLevel::FATAL)
            std::cerr << "[FATAL] [" << time_str << "] " << formatted_message << std::endl;
          break;
        default:
          break;
      }
      
      // 同时写入终端日志文件
      if (enableTerminalLogFile_ && terminalLogStream_.is_open())
      {
        std::lock_guard<std::mutex> lock(mutex_);
        try
        {
          // 获取当前时间
          auto now = std::chrono::system_clock::now();
          auto now_time_t = std::chrono::system_clock::to_time_t(now);
          auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
              now.time_since_epoch()) % 1000;
          
          // 使用线程安全的时间转换函数（C++14兼容）
          std::tm now_tm;
          #ifdef _WIN32
            localtime_s(&now_tm, &now_time_t);
          #else
            localtime_r(&now_time_t, &now_tm);
          #endif
          
          // 格式化时间字符串 - 使用北京时间格式（去掉年份）
    char time_str[50];
    std::snprintf(time_str, sizeof(time_str), "%02d-%02d %02d:%02d:%02d.%03d",
                now_tm.tm_mon + 1,
                now_tm.tm_mday,
                now_tm.tm_hour,
                now_tm.tm_min,
                now_tm.tm_sec,
                static_cast<int>(now_ms.count()));
          time_str[sizeof(time_str) - 1] = '\0';
          
          // 格式化日志级别
          std::string level_str = getLevelString(level);
          
          // 写入终端日志文件
          terminalLogStream_ << "[" << time_str << "] [" << level_str << "] " << formatted_message << std::endl;
          
          // 刷新文件流
          terminalLogStream_.flush();
        }
        catch (const std::exception& e)
        {
          std::cerr << "Failed to write log to terminal log file: " << e.what() << std::endl;
        }
      }
    }
    
    // 如果启用了文件日志，写入日志文件
    if (enableFileLog_)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      
      try
      {
        if (logFileStream_.is_open())
        {
          // 获取当前时间
          auto now = std::chrono::system_clock::now();
          auto now_time_t = std::chrono::system_clock::to_time_t(now);
          auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
              now.time_since_epoch()) % 1000;
          
          // 使用线程安全的时间转换函数（C++14兼容）
          std::tm now_tm;
          #ifdef _WIN32
            localtime_s(&now_tm, &now_time_t);
          #else
            localtime_r(&now_time_t, &now_tm);
          #endif
          
          // 格式化时间字符串 - 使用北京时间格式（去掉年份）
          char time_str[50];
          std::snprintf(time_str, sizeof(time_str), "%02d-%02d %02d:%02d:%02d.%03d",
                      now_tm.tm_mon + 1,
                      now_tm.tm_mday,
                      now_tm.tm_hour,
                      now_tm.tm_min,
                      now_tm.tm_sec,
                      static_cast<int>(now_ms.count()));
          time_str[sizeof(time_str) - 1] = '\0'; // 确保字符串以null结尾
          
          // 格式化日志级别
          std::string level_str = getLevelString(level);
          
          // 写入日志文件
          logFileStream_ << "[" << time_str << "] [" << level_str << "] " << formatted_message << std::endl;
          
          // 刷新文件流，确保日志立即写入
          logFileStream_.flush();
        }
      }
      catch (const std::exception& e)
      {
        std::cerr << "Failed to write log to file: " << e.what() << std::endl;
      }
    }
  }

  // 基本配置
  LogLevel logLevel_;              // 当前日志级别
  bool enableConsoleLog_;          // 是否启用控制台日志
  bool enableFileLog_;             // 是否启用文件日志
  bool enableTerminalLogFile_;     // 是否启用终端日志文件
  
  // 文件日志配置
  std::string logFilePath_;        // 日志文件基础路径
  std::ofstream logFileStream_;    // 日志文件流
  
  // 终端日志文件配置
  std::string terminalLogFilePath_; // 终端日志文件路径
  std::ofstream terminalLogStream_; // 终端日志文件流
  
  mutable std::mutex mutex_;       // 互斥锁，保证线程安全
};

/**
 * @brief 日志宏定义，方便使用
 */
#define PSDK_DEBUG(tag, ...)   dji_psdk_wrapper::Logger::getInstance().debug(tag, __VA_ARGS__)
#define PSDK_INFO(tag, ...)    dji_psdk_wrapper::Logger::getInstance().info(tag, __VA_ARGS__)
#define PSDK_WARNING(tag, ...) dji_psdk_wrapper::Logger::getInstance().warning(tag, __VA_ARGS__)
#define PSDK_ERROR(tag, ...)   dji_psdk_wrapper::Logger::getInstance().error(tag, __VA_ARGS__)
#define PSDK_FATAL(tag, ...)   dji_psdk_wrapper::Logger::getInstance().fatal(tag, __VA_ARGS__)

}  // namespace dji_psdk_wrapper

#endif  // DJI_PSDK_WRAPPER_LOGGER_HPP