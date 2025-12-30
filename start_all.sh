#!/bin/bash

# 一键启动脚本：启动HTTP服务、rosbridge、无人机节点和地面站节点

echo "=== 正在启动ROS2智能吊装系统所有服务 ==="

# 设置工作目录（使用相对路径，自动获取脚本所在目录）
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
HTTP_DIR="${WORKSPACE_DIR}/src/ground_station/resources"

# 检查是否已激活ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "[1/5] 激活ROS 2环境..."
    source /opt/ros/jazzy/setup.bash
fi
# 始终加载本地包的setup文件
source "${WORKSPACE_DIR}/install/local_setup.bash"
echo "[1/5] ROS 2环境已激活 (${ROS_DISTRO})"

# 创建日志目录
LOG_DIR="${WORKSPACE_DIR}/userLog"
mkdir -p "${LOG_DIR}"

# 启动HTTP服务
echo "[2/5] 启动HTTP服务..."
cd "${HTTP_DIR}" && python3 -m http.server 8080 > "${LOG_DIR}/http.log" 2>&1 &
HTTP_PID=$!
echo "      HTTP服务已启动，PID: ${HTTP_PID}"

# 启动rosbridge服务
echo "[3/5] 启动rosbridge服务..."
/opt/ros/jazzy/lib/rosbridge_server/rosbridge_websocket > "${LOG_DIR}/rosbridge.log" 2>&1 &
ROSBRIDGE_PID=$!
echo "      rosbridge服务已启动，PID: ${ROSBRIDGE_PID}"

# 启动无人机节点（dji_psdk_node）
echo "[4/5] 启动无人机节点..."
ros2 run dji_psdk_wrapper dji_psdk_node > "${LOG_DIR}/dji_psdk_node.log" 2>&1 &
DRONE_PID=$!
echo "      无人机节点已启动，PID: ${DRONE_PID}"

# 启动地面站节点
echo "[5/5] 启动地面站节点..."
ros2 run ground_station ground_station_node > "${LOG_DIR}/ground_station_node.log" 2>&1 &
GROUND_STATION_PID=$!
echo "      地面站节点已启动，PID: ${GROUND_STATION_PID}"

# 保存PID到文件
cat > "${WORKSPACE_DIR}/all_services.pid" << EOF
HTTP_PID=${HTTP_PID}
ROSBRIDGE_PID=${ROSBRIDGE_PID}
DRONE_PID=${DRONE_PID}
GROUND_STATION_PID=${GROUND_STATION_PID}
EOF

echo "=== 所有服务启动完成！ ==="
echo ""
echo "服务列表："
echo "- HTTP服务：http://localhost:8080"
echo "- rosbridge：ws://localhost:9090"
echo "- 无人机节点：dji_psdk_node"
echo "- 地面站节点：ground_station_node"
echo ""
echo "停止所有服务：运行 ./stop_all.sh"
echo "查看服务日志："
echo "  - HTTP服务: cat ${LOG_DIR}/http.log"
echo "  - rosbridge: cat ${LOG_DIR}/rosbridge.log"
echo "  - 无人机节点: cat ${LOG_DIR}/dji_psdk_node.log"
echo "  - 地面站节点: cat ${LOG_DIR}/ground_station_node.log"