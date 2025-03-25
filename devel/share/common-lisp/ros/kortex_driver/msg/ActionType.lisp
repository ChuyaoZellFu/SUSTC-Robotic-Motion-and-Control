; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ActionType.msg.html

(cl:defclass <ActionType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ActionType (<ActionType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActionType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActionType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ActionType> is deprecated: use kortex_driver-msg:ActionType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ActionType>)))
    "Constants for message type '<ActionType>"
  '((:UNSPECIFIED_ACTION . 0)
    (:SEND_TWIST_COMMAND . 1)
    (:SEND_WRENCH_COMMAND . 2)
    (:SEND_JOINT_SPEEDS . 4)
    (:REACH_POSE . 6)
    (:REACH_JOINT_ANGLES . 7)
    (:TOGGLE_ADMITTANCE_MODE . 13)
    (:SNAPSHOT . 14)
    (:SWITCH_CONTROL_MAPPING . 16)
    (:NAVIGATE_JOINTS . 17)
    (:NAVIGATE_MAPPINGS . 18)
    (:CHANGE_TWIST . 22)
    (:CHANGE_JOINT_SPEEDS . 23)
    (:CHANGE_WRENCH . 25)
    (:APPLY_EMERGENCY_STOP . 28)
    (:CLEAR_FAULTS . 29)
    (:TIME_DELAY . 31)
    (:EXECUTE_ACTION . 32)
    (:SEND_GRIPPER_COMMAND . 33)
    (:SEND_GPIO_COMMAND . 34)
    (:STOP_ACTION . 35)
    (:PLAY_PRE_COMPUTED_TRAJECTORY . 39)
    (:EXECUTE_SEQUENCE . 40)
    (:EXECUTE_WAYPOINT_LIST . 41))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ActionType)))
    "Constants for message type 'ActionType"
  '((:UNSPECIFIED_ACTION . 0)
    (:SEND_TWIST_COMMAND . 1)
    (:SEND_WRENCH_COMMAND . 2)
    (:SEND_JOINT_SPEEDS . 4)
    (:REACH_POSE . 6)
    (:REACH_JOINT_ANGLES . 7)
    (:TOGGLE_ADMITTANCE_MODE . 13)
    (:SNAPSHOT . 14)
    (:SWITCH_CONTROL_MAPPING . 16)
    (:NAVIGATE_JOINTS . 17)
    (:NAVIGATE_MAPPINGS . 18)
    (:CHANGE_TWIST . 22)
    (:CHANGE_JOINT_SPEEDS . 23)
    (:CHANGE_WRENCH . 25)
    (:APPLY_EMERGENCY_STOP . 28)
    (:CLEAR_FAULTS . 29)
    (:TIME_DELAY . 31)
    (:EXECUTE_ACTION . 32)
    (:SEND_GRIPPER_COMMAND . 33)
    (:SEND_GPIO_COMMAND . 34)
    (:STOP_ACTION . 35)
    (:PLAY_PRE_COMPUTED_TRAJECTORY . 39)
    (:EXECUTE_SEQUENCE . 40)
    (:EXECUTE_WAYPOINT_LIST . 41))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActionType>) ostream)
  "Serializes a message object of type '<ActionType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActionType>) istream)
  "Deserializes a message object of type '<ActionType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActionType>)))
  "Returns string type for a message object of type '<ActionType>"
  "kortex_driver/ActionType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActionType)))
  "Returns string type for a message object of type 'ActionType"
  "kortex_driver/ActionType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActionType>)))
  "Returns md5sum for a message object of type '<ActionType>"
  "2814fa62baddf71292e9d832616cc515")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActionType)))
  "Returns md5sum for a message object of type 'ActionType"
  "2814fa62baddf71292e9d832616cc515")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActionType>)))
  "Returns full string definition for message of type '<ActionType>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_ACTION = 0~%~%uint32 SEND_TWIST_COMMAND = 1~%~%uint32 SEND_WRENCH_COMMAND = 2~%~%uint32 SEND_JOINT_SPEEDS = 4~%~%uint32 REACH_POSE = 6~%~%uint32 REACH_JOINT_ANGLES = 7~%~%uint32 TOGGLE_ADMITTANCE_MODE = 13~%~%uint32 SNAPSHOT = 14~%~%uint32 SWITCH_CONTROL_MAPPING = 16~%~%uint32 NAVIGATE_JOINTS = 17~%~%uint32 NAVIGATE_MAPPINGS = 18~%~%uint32 CHANGE_TWIST = 22~%~%uint32 CHANGE_JOINT_SPEEDS = 23~%~%uint32 CHANGE_WRENCH = 25~%~%uint32 APPLY_EMERGENCY_STOP = 28~%~%uint32 CLEAR_FAULTS = 29~%~%uint32 TIME_DELAY = 31~%~%uint32 EXECUTE_ACTION = 32~%~%uint32 SEND_GRIPPER_COMMAND = 33~%~%uint32 SEND_GPIO_COMMAND = 34~%~%uint32 STOP_ACTION = 35~%~%uint32 PLAY_PRE_COMPUTED_TRAJECTORY = 39~%~%uint32 EXECUTE_SEQUENCE = 40~%~%uint32 EXECUTE_WAYPOINT_LIST = 41~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActionType)))
  "Returns full string definition for message of type 'ActionType"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_ACTION = 0~%~%uint32 SEND_TWIST_COMMAND = 1~%~%uint32 SEND_WRENCH_COMMAND = 2~%~%uint32 SEND_JOINT_SPEEDS = 4~%~%uint32 REACH_POSE = 6~%~%uint32 REACH_JOINT_ANGLES = 7~%~%uint32 TOGGLE_ADMITTANCE_MODE = 13~%~%uint32 SNAPSHOT = 14~%~%uint32 SWITCH_CONTROL_MAPPING = 16~%~%uint32 NAVIGATE_JOINTS = 17~%~%uint32 NAVIGATE_MAPPINGS = 18~%~%uint32 CHANGE_TWIST = 22~%~%uint32 CHANGE_JOINT_SPEEDS = 23~%~%uint32 CHANGE_WRENCH = 25~%~%uint32 APPLY_EMERGENCY_STOP = 28~%~%uint32 CLEAR_FAULTS = 29~%~%uint32 TIME_DELAY = 31~%~%uint32 EXECUTE_ACTION = 32~%~%uint32 SEND_GRIPPER_COMMAND = 33~%~%uint32 SEND_GPIO_COMMAND = 34~%~%uint32 STOP_ACTION = 35~%~%uint32 PLAY_PRE_COMPUTED_TRAJECTORY = 39~%~%uint32 EXECUTE_SEQUENCE = 40~%~%uint32 EXECUTE_WAYPOINT_LIST = 41~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActionType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActionType>))
  "Converts a ROS message object to a list"
  (cl:list 'ActionType
))
