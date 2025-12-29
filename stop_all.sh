#!/bin/bash

# 一键停止脚本：停止HTTP服务、rosbridge、无人机节点和地面站节点

echo "=== 正在停止ROS2智能吊装系统所有服务 ==="

# 设置工作目录
WORKSPACE_DIR="/home/lcf/ros2_hoisting_system"
PID_FILE="${WORKSPACE_DIR}/all_services.pid"

# 检查PID文件是否存在
if [ ! -f "${PID_FILE}" ]; then
    echo "错误：未找到服务PID文件 (${PID_FILE})"
    echo "请先运行 ./start_all.sh 启动服务"
    exit 1
fi

# 读取PID文件
source "${PID_FILE}"

# 停止地面站节点
echo "[1/4] 停止地面站节点..."
if [ -n "${GROUND_STATION_PID}" ]; then
    kill "${GROUND_STATION_PID}" > /dev/null 2>&1
    echo "      地面站节点已停止 (PID: ${GROUND_STATION_PID})"
fi

# 停止无人机节点
echo "[2/4] 停止无人机节点..."
if [ -n "${DRONE_PID}" ]; then
    kill "${DRONE_PID}" > /dev/null 2>&1
    echo "      无人机节点已停止 (PID: ${DRONE_PID})"
fi

# 停止rosbridge服务
echo "[3/4] 停止rosbridge服务..."
if [ -n "${ROSBRIDGE_PID}" ]; then
    kill "${ROSBRIDGE_PID}" > /dev/null 2>&1
    echo "      rosbridge服务已停止 (PID: ${ROSBRIDGE_PID})"
fi

# 停止HTTP服务
echo "[4/4] 停止HTTP服务..."
if [ -n "${HTTP_PID}" ]; then
    kill "${HTTP_PID}" > /dev/null 2>&1
    echo "      HTTP服务已停止 (PID: ${HTTP_PID})"
fi

# 删除PID文件
rm -f "${PID_FILE}"

echo "=== 所有服务已停止！ ==="