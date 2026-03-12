# CMakeLists.txt 配置说明

为了使用自定义消息类型，需要在 CMakeLists.txt 中添加以下内容：

## 1. 在 find_package 中添加：

```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation  # 添加这一行
)
```

## 2. 添加消息文件：

```cmake
## 生成消息
add_message_files(
  FILES
  ImpedanceParams.msg
  JointPositionCommand.msg
)

## 生成消息依赖
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
```

## 3. 在 catkin_package 中添加：

```cmake
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    roscpp 
    std_msgs 
    geometry_msgs 
    sensor_msgs
    message_runtime  # 添加这一行
)
```

## 4. 安装Python脚本：

```cmake
## 标记可执行Python脚本
catkin_install_python(PROGRAMS
  scripts/cartesian_impedance_controller_node.py
  scripts/test_impedance_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

## 5. 安装配置文件和launch文件：

```cmake
## 安装launch文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## 安装配置文件
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
```

## 完整示例

如果您的 CMakeLists.txt 需要更新，可以参考以下完整示例结构：

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(afp_robot_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

## 生成消息
add_message_files(
  FILES
  ImpedanceParams.msg
  JointPositionCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    geometry_msgs 
    sensor_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 安装Python脚本
catkin_install_python(PROGRAMS
  scripts/cartesian_impedance_controller_node.py
  scripts/test_impedance_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装launch和config
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
```

## 编译步骤

```bash
cd ~/Project/AFP
catkin_make
source devel/setup.bash
```

## 验证消息是否生成

```bash
rosmsg show afp_robot_control/ImpedanceParams
rosmsg show afp_robot_control/JointPositionCommand
```
