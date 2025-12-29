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

#ifndef DJI_PSDK_WRAPPER_CONFIG_MANAGER_HPP
#define DJI_PSDK_WRAPPER_CONFIG_MANAGER_HPP

#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <chrono>
#include <sstream>

namespace dji_psdk_wrapper
{

/**
 * @brief 配置管理器类
 * 
 * 负责加载、解析和管理YAML配置文件
 * 支持复杂的配置结构，包括嵌套配置、数组、字典等
 * 提供线程安全的配置访问接口
 */
class ConfigManager
{
public:
  /**
   * @brief 获取配置管理器单例实例
   * @return ConfigManager& 配置管理器实例
   */
  static ConfigManager& getInstance()
  {
    static ConfigManager instance;
    return instance;
  }

  /**
   * @brief 加载配置文件
   * @param configPath 配置文件路径
   * @return bool 是否加载成功
   */
  bool loadConfig(const std::string& configPath)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    try
    {
      // 检查文件是否存在
      std::ifstream file(configPath);
      if (!file.good())
      {
        return false;
      }
      file.close();
      
      config_ = YAML::LoadFile(configPath);
      configPath_ = configPath;
      loaded_ = true;
      
      // 记录配置文件信息
      configStats_.lastLoadTime = std::chrono::system_clock::now();
      configStats_.loadCount++;
      
      return true;
    }
    catch (const YAML::Exception& e)
    {
      configStats_.errorCount++;
      lastError_ = std::string("YAML解析错误: ") + e.what();
      return false;
    }
    catch (const std::exception& e)
    {
      configStats_.errorCount++;
      lastError_ = std::string("加载配置文件错误: ") + e.what();
      return false;
    }
  }

  /**
   * @brief 重新加载配置文件
   * @return bool 是否重新加载成功
   */
  bool reloadConfig()
  {
    if (configPath_.empty())
    {
      lastError_ = "配置文件路径为空";
      return false;
    }
    return loadConfig(configPath_);
  }

  /**
   * @brief 从字符串加载配置
   * @param configContent 配置内容字符串
   * @return bool 是否加载成功
   */
  bool loadConfigFromString(const std::string& configContent)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    try
    {
      config_ = YAML::Load(configContent);
      configPath_ = "string_content";
      loaded_ = true;
      
      configStats_.lastLoadTime = std::chrono::system_clock::now();
      configStats_.loadCount++;
      
      return true;
    }
    catch (const YAML::Exception& e)
    {
      configStats_.errorCount++;
      lastError_ = std::string("YAML解析错误: ") + e.what();
      return false;
    }
    catch (const std::exception& e)
    {
      configStats_.errorCount++;
      lastError_ = std::string("加载配置字符串错误: ") + e.what();
      return false;
    }
  }

  /**
   * @brief 获取字符串配置项
   * @param key 配置项键名
   * @param defaultValue 默认值
   * @return std::string 配置值
   */
  std::string getString(const std::string& key, const std::string& defaultValue = "") const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!loaded_)
    {
      configStats_.missCount++;
      return defaultValue;
    }
    
    try
    {
      auto value = getValueFromKey(key);
      configStats_.hitCount++;
      return value.as<std::string>();
    }
    catch (const std::exception&)
    {
      configStats_.missCount++;
      return defaultValue;
    }
  }

  /**
   * @brief 获取整数配置项
   * @param key 配置项键名
   * @param defaultValue 默认值
   * @return int 配置值
   */
  int getInt(const std::string& key, int defaultValue = 0) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!loaded_)
    {
      configStats_.missCount++;
      return defaultValue;
    }
    
    try
    {
      auto value = getValueFromKey(key);
      configStats_.hitCount++;
      return value.as<int>();
    }
    catch (const std::exception&)
    {
      configStats_.missCount++;
      return defaultValue;
    }
  }

  /**
   * @brief 获取浮点数配置项
   * @param key 配置项键名
   * @param defaultValue 默认值
   * @return double 配置值
   */
  double getDouble(const std::string& key, double defaultValue = 0.0) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!loaded_)
    {
      configStats_.missCount++;
      return defaultValue;
    }
    
    try
    {
      auto value = getValueFromKey(key);
      configStats_.hitCount++;
      return value.as<double>();
    }
    catch (const std::exception&)
    {
      configStats_.missCount++;
      return defaultValue;
    }
  }

  /**
   * @brief 获取布尔值配置项
   * @param key 配置项键名
   * @param defaultValue 默认值
   * @return bool 配置值
   */
  bool getBool(const std::string& key, bool defaultValue = false) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!loaded_)
    {
      configStats_.missCount++;
      return defaultValue;
    }
    
    try
    {
      auto value = getValueFromKey(key);
      configStats_.hitCount++;
      return value.as<bool>();
    }
    catch (const std::exception&)
    {
      configStats_.missCount++;
      return defaultValue;
    }
  }

  /**
   * @brief 检查配置项是否存在
   * @param key 配置项键名
   * @return bool 是否存在
   */
  bool hasKey(const std::string& key) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!loaded_)
    {
      return false;
    }
    
    try
    {
      getValueFromKey(key);
      return true;
    }
    catch (const std::exception&)
    {
      return false;
    }
  }

  /**
   * @brief 获取字符串数组配置项
   * @param key 配置项键名
   * @param defaultValue 默认值
   * @return std::vector<std::string> 配置值数组
   */
  std::vector<std::string> getStringArray(const std::string& key, 
                                         const std::vector<std::string>& defaultValue = {}) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!loaded_)
    {
      configStats_.missCount++;
      return defaultValue;
    }
    
    try
    {
      auto value = getValueFromKey(key);
      if (!value.IsSequence())
      {
        configStats_.missCount++;
        return defaultValue;
      }
      
      std::vector<std::string> result;
      for (const auto& item : value)
      {
        result.push_back(item.as<std::string>());
      }
      
      configStats_.hitCount++;
      return result;
    }
    catch (const std::exception&)
    {
      configStats_.missCount++;
      return defaultValue;
    }
  }

  /**
   * @brief 获取整数数组配置项
   * @param key 配置项键名
   * @param defaultValue 默认值
   * @return std::vector<int> 配置值数组
   */
  std::vector<int> getIntArray(const std::string& key, 
                              const std::vector<int>& defaultValue = {}) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!loaded_)
    {
      configStats_.missCount++;
      return defaultValue;
    }
    
    try
    {
      auto value = getValueFromKey(key);
      if (!value.IsSequence())
      {
        configStats_.missCount++;
        return defaultValue;
      }
      
      std::vector<int> result;
      for (const auto& item : value)
      {
        result.push_back(item.as<int>());
      }
      
      configStats_.hitCount++;
      return result;
    }
    catch (const std::exception&)
    {
      configStats_.missCount++;
      return defaultValue;
    }
  }

  /**
   * @brief 获取嵌套配置节点
   * @param key 配置项键名
   * @return YAML::Node 嵌套配置节点
   */
  YAML::Node getNode(const std::string& key) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!loaded_)
    {
      configStats_.missCount++;
      return YAML::Node();
    }
    
    try
    {
      auto value = getValueFromKey(key);
      configStats_.hitCount++;
      return value;
    }
    catch (const std::exception&)
    {
      configStats_.missCount++;
      return YAML::Node();
    }
  }

  /**
   * @brief 获取配置文件路径
   * @return std::string 配置文件路径
   */
  std::string getConfigPath() const
  {
    return configPath_;
  }

  /**
   * @brief 检查配置是否已加载
   * @return bool 是否已加载
   */
  bool isLoaded() const
  {
    return loaded_;
  }

  /**
   * @brief 获取最后一次错误信息
   * @return std::string 错误信息
   */
  std::string getLastError() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return lastError_;
  }

  /**
   * @brief 获取配置管理器统计信息
   * @return std::string 统计信息
   */
  std::string getStats() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::stringstream ss;
    ss << "配置管理器统计信息:\n"
       << "- 配置文件: " << configPath_ << "\n"
       << "- 是否已加载: " << (loaded_ ? "是" : "否") << "\n"
       << "- 加载次数: " << configStats_.loadCount << "\n"
       << "- 成功查询次数: " << configStats_.hitCount << "\n"
       << "- 失败查询次数: " << configStats_.missCount << "\n"
       << "- 错误次数: " << configStats_.errorCount << "\n";
    
    if (configStats_.lastLoadTime.time_since_epoch().count() > 0)
    {
      auto now = std::chrono::system_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::seconds>(
          now - configStats_.lastLoadTime);
      ss << "- 最后加载时间: " << duration.count() << "秒前\n";
    }
    
    return ss.str();
  }

  /**
   * @brief 重置统计信息
   */
  void resetStats()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    configStats_ = ConfigStats();
    lastError_.clear();
  }

private:
  ConfigManager() : loaded_(false) {}
  
  ~ConfigManager() {}
  
  /**
   * @brief 配置统计信息结构体
   */
  struct ConfigStats
  {
    uint64_t loadCount = 0;              // 配置加载次数
    uint64_t hitCount = 0;               // 成功查询次数
    uint64_t missCount = 0;              // 失败查询次数
    uint64_t errorCount = 0;             // 错误次数
    std::chrono::system_clock::time_point lastLoadTime;  // 最后加载时间
  };

  /**
   * @brief 从键名获取YAML值
   * @param key 键名，支持点分隔符（如 "logger.level"）
   * @return YAML::Node YAML节点
   */
  YAML::Node getValueFromKey(const std::string& key) const
  {
    std::vector<std::string> keys = splitKey(key);
    YAML::Node current = config_;
    
    for (const auto& k : keys)
    {
      if (!current[k])
      {
        throw std::runtime_error("Key not found: " + key);
      }
      current = current[k];
    }
    
    return current;
  }

  /**
   * @brief 分割键名
   * @param key 键名
   * @return std::vector<std::string> 分割后的键名列表
   */
  std::vector<std::string> splitKey(const std::string& key) const
  {
    std::vector<std::string> result;
    std::stringstream ss(key);
    std::string token;
    
    while (std::getline(ss, token, '.'))
    {
      if (!token.empty())
      {
        result.push_back(token);
      }
    }
    
    return result;
  }

  YAML::Node config_;          // YAML配置节点
  std::string configPath_;     // 配置文件路径
  bool loaded_;                // 是否已加载配置
  mutable std::string lastError_;      // 最后一次错误信息
  mutable ConfigStats configStats_;    // 配置统计信息
  mutable std::mutex mutex_;   // 互斥锁
};

/**
 * @brief 配置宏定义，方便使用
 */
#define CONFIG_GET_STRING(key, default) \
  dji_psdk_wrapper::ConfigManager::getInstance().getString(key, default)

#define CONFIG_GET_INT(key, default) \
  dji_psdk_wrapper::ConfigManager::getInstance().getInt(key, default)

#define CONFIG_GET_DOUBLE(key, default) \
  dji_psdk_wrapper::ConfigManager::getInstance().getDouble(key, default)

#define CONFIG_GET_BOOL(key, default) \
  dji_psdk_wrapper::ConfigManager::getInstance().getBool(key, default)

#define CONFIG_HAS_KEY(key) \
  dji_psdk_wrapper::ConfigManager::getInstance().hasKey(key)

}  // namespace dji_psdk_wrapper

#endif  // DJI_PSDK_WRAPPER_CONFIG_MANAGER_HPP