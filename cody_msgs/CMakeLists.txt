cmake_minimum_required(VERSION 3.8)
project(cody_msgs)
# 1.在find_package中添加message_generation
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)
 
# 2.add_message_files中添加自定义的msg文件
add_message_files(
        #DIRECTORY msg
        FILES
        CtrlCmd.msg
        OdomFb.msg
        SystemStateFb.msg
        MotorMotionFb.msg
)
 
# 3.生成消息时依赖于 std_msgs
generate_messages(
        DEPENDENCIES
        std_msgs
)
 
# 4.执行时依赖
catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)
 
# 5.头文件目录
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

