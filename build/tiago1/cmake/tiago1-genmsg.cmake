# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tiago1: 22 messages, 2 services")

set(MSG_I_FLAGS "-Itiago1:/root/cogar_ws/src/tiago1/msg;-Itiago1:/root/cogar_ws/devel/share/tiago1/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tiago1_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/cogar_ws/src/tiago1/msg/Voice_rec.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/src/tiago1/msg/Voice_rec.msg" ""
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlAction.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/MovementControlAction.msg" "tiago1/MovementControlActionResult:nav_msgs/Path:tiago1/MovementControlResult:actionlib_msgs/GoalID:geometry_msgs/Quaternion:tiago1/MovementControlFeedback:actionlib_msgs/GoalStatus:std_msgs/Header:tiago1/MovementControlActionFeedback:geometry_msgs/Pose:geometry_msgs/Point:tiago1/MovementControlActionGoal:geometry_msgs/PoseStamped:tiago1/MovementControlGoal"
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionGoal.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionGoal.msg" "nav_msgs/Path:actionlib_msgs/GoalID:geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Pose:geometry_msgs/Point:tiago1/MovementControlGoal:geometry_msgs/PoseStamped"
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionResult.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionResult.msg" "actionlib_msgs/GoalStatus:tiago1/MovementControlResult:std_msgs/Header:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionFeedback.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionFeedback.msg" "actionlib_msgs/GoalStatus:tiago1/MovementControlFeedback:std_msgs/Header:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg" "nav_msgs/Path:geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Point:geometry_msgs/Pose:geometry_msgs/PoseStamped"
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg" ""
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg" ""
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlAction.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/ArmControlAction.msg" "tiago1/ArmControlFeedback:tiago1/ArmControlResult:actionlib_msgs/GoalID:tiago1/ArmControlActionFeedback:tiago1/ArmControlGoal:actionlib_msgs/GoalStatus:std_msgs/Header:tiago1/ArmControlActionResult:tiago1/ArmControlActionGoal"
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionGoal.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionGoal.msg" "tiago1/ArmControlGoal:std_msgs/Header:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionResult.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionResult.msg" "actionlib_msgs/GoalStatus:tiago1/ArmControlResult:std_msgs/Header:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionFeedback.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionFeedback.msg" "tiago1/ArmControlFeedback:actionlib_msgs/GoalStatus:std_msgs/Header:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg" ""
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg" ""
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg" ""
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlAction.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/GripperControlAction.msg" "actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:tiago1/GripperControlFeedback:std_msgs/Header:tiago1/GripperControlGoal:tiago1/GripperControlActionGoal:tiago1/GripperControlActionResult:tiago1/GripperControlResult:tiago1/GripperControlActionFeedback"
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionGoal.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionGoal.msg" "std_msgs/Header:actionlib_msgs/GoalID:tiago1/GripperControlGoal"
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionResult.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionResult.msg" "actionlib_msgs/GoalStatus:tiago1/GripperControlResult:std_msgs/Header:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionFeedback.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionFeedback.msg" "actionlib_msgs/GoalStatus:tiago1/GripperControlFeedback:std_msgs/Header:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg" ""
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg" ""
)

get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg" ""
)

get_filename_component(_filename "/root/cogar_ws/src/tiago1/srv/send_order.srv" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/src/tiago1/srv/send_order.srv" "tiago1/Voice_rec"
)

get_filename_component(_filename "/root/cogar_ws/src/tiago1/srv/robotstatedecision.srv" NAME_WE)
add_custom_target(_tiago1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago1" "/root/cogar_ws/src/tiago1/srv/robotstatedecision.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tiago1
  "/root/cogar_ws/src/tiago1/msg/Voice_rec.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlAction.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionResult.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionFeedback.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlAction.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionFeedback.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionResult.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionGoal.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionResult.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_msg_cpp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)

### Generating Services
_generate_srv_cpp(tiago1
  "/root/cogar_ws/src/tiago1/srv/send_order.srv"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/src/tiago1/msg/Voice_rec.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)
_generate_srv_cpp(tiago1
  "/root/cogar_ws/src/tiago1/srv/robotstatedecision.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
)

### Generating Module File
_generate_module_cpp(tiago1
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tiago1_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tiago1_generate_messages tiago1_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/cogar_ws/src/tiago1/msg/Voice_rec.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlAction.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlAction.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlAction.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/src/tiago1/srv/send_order.srv" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/src/tiago1/srv/robotstatedecision.srv" NAME_WE)
add_dependencies(tiago1_generate_messages_cpp _tiago1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tiago1_gencpp)
add_dependencies(tiago1_gencpp tiago1_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tiago1_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tiago1
  "/root/cogar_ws/src/tiago1/msg/Voice_rec.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlAction.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionResult.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionFeedback.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlAction.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionFeedback.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionResult.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionGoal.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionResult.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_msg_eus(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)

### Generating Services
_generate_srv_eus(tiago1
  "/root/cogar_ws/src/tiago1/srv/send_order.srv"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/src/tiago1/msg/Voice_rec.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)
_generate_srv_eus(tiago1
  "/root/cogar_ws/src/tiago1/srv/robotstatedecision.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
)

### Generating Module File
_generate_module_eus(tiago1
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tiago1_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tiago1_generate_messages tiago1_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/cogar_ws/src/tiago1/msg/Voice_rec.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlAction.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlAction.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlAction.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/src/tiago1/srv/send_order.srv" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/src/tiago1/srv/robotstatedecision.srv" NAME_WE)
add_dependencies(tiago1_generate_messages_eus _tiago1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tiago1_geneus)
add_dependencies(tiago1_geneus tiago1_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tiago1_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tiago1
  "/root/cogar_ws/src/tiago1/msg/Voice_rec.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlAction.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionResult.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionFeedback.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlAction.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionFeedback.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionResult.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionGoal.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionResult.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_msg_lisp(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)

### Generating Services
_generate_srv_lisp(tiago1
  "/root/cogar_ws/src/tiago1/srv/send_order.srv"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/src/tiago1/msg/Voice_rec.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)
_generate_srv_lisp(tiago1
  "/root/cogar_ws/src/tiago1/srv/robotstatedecision.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
)

### Generating Module File
_generate_module_lisp(tiago1
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tiago1_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tiago1_generate_messages tiago1_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/cogar_ws/src/tiago1/msg/Voice_rec.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlAction.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlAction.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlAction.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/src/tiago1/srv/send_order.srv" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/src/tiago1/srv/robotstatedecision.srv" NAME_WE)
add_dependencies(tiago1_generate_messages_lisp _tiago1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tiago1_genlisp)
add_dependencies(tiago1_genlisp tiago1_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tiago1_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/src/tiago1/msg/Voice_rec.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlAction.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionResult.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionFeedback.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlAction.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionFeedback.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionResult.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionGoal.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionResult.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_msg_nodejs(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)

### Generating Services
_generate_srv_nodejs(tiago1
  "/root/cogar_ws/src/tiago1/srv/send_order.srv"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/src/tiago1/msg/Voice_rec.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)
_generate_srv_nodejs(tiago1
  "/root/cogar_ws/src/tiago1/srv/robotstatedecision.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
)

### Generating Module File
_generate_module_nodejs(tiago1
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tiago1_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tiago1_generate_messages tiago1_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/cogar_ws/src/tiago1/msg/Voice_rec.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlAction.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlAction.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlAction.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/src/tiago1/srv/send_order.srv" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/src/tiago1/srv/robotstatedecision.srv" NAME_WE)
add_dependencies(tiago1_generate_messages_nodejs _tiago1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tiago1_gennodejs)
add_dependencies(tiago1_gennodejs tiago1_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tiago1_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tiago1
  "/root/cogar_ws/src/tiago1/msg/Voice_rec.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlAction.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionResult.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionFeedback.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlAction.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionFeedback.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionResult.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionGoal.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionResult.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_msg_py(tiago1
  "/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)

### Generating Services
_generate_srv_py(tiago1
  "/root/cogar_ws/src/tiago1/srv/send_order.srv"
  "${MSG_I_FLAGS}"
  "/root/cogar_ws/src/tiago1/msg/Voice_rec.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)
_generate_srv_py(tiago1
  "/root/cogar_ws/src/tiago1/srv/robotstatedecision.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
)

### Generating Module File
_generate_module_py(tiago1
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tiago1_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tiago1_generate_messages tiago1_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/cogar_ws/src/tiago1/msg/Voice_rec.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlAction.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlActionFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/MovementControlFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlAction.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlActionFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/ArmControlFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlAction.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlActionFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlGoal.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlResult.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/devel/share/tiago1/msg/GripperControlFeedback.msg" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/src/tiago1/srv/send_order.srv" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/cogar_ws/src/tiago1/srv/robotstatedecision.srv" NAME_WE)
add_dependencies(tiago1_generate_messages_py _tiago1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tiago1_genpy)
add_dependencies(tiago1_genpy tiago1_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tiago1_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago1
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tiago1_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(tiago1_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(tiago1_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago1
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tiago1_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(tiago1_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(tiago1_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago1
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tiago1_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(tiago1_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(tiago1_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago1
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tiago1_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(tiago1_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(tiago1_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago1
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tiago1_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(tiago1_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(tiago1_generate_messages_py nav_msgs_generate_messages_py)
endif()
