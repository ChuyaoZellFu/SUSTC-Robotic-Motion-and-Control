; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude TrajectoryInfoType.msg.html

(cl:defclass <TrajectoryInfoType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass TrajectoryInfoType (<TrajectoryInfoType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryInfoType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryInfoType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<TrajectoryInfoType> is deprecated: use kortex_driver-msg:TrajectoryInfoType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<TrajectoryInfoType>)))
    "Constants for message type '<TrajectoryInfoType>"
  '((:UNSPECIFIED_TRAJECTORY_INFORMATION . 0)
    (:JOINT_ACCELERATION_LIMIT_REACHED . 1)
    (:JOINT_SPEED_LIMIT_REACHED . 2)
    (:JOINT_POSITION_LIMIT_REACHED . 3)
    (:JOINT_TORQUE_LIMIT_REACHED . 4)
    (:SINGULARITY_REGION . 5)
    (:INVERSE_KINEMATIC_FAILED . 6)
    (:CARTESIAN_ACCELERATION_LIMIT_REACHED . 7)
    (:CARTESIAN_SPEED_LIMIT_REACHED . 8)
    (:CARTESIAN_POSITION_LIMIT_REACHED . 9)
    (:CARTESIAN_WRENCH_LIMIT_REACHED . 10)
    (:ENTERING_PROTECTION_ZONE . 11)
    (:WAYPOINT_REACHED . 12)
    (:TRAJECTORY_OK . 13))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'TrajectoryInfoType)))
    "Constants for message type 'TrajectoryInfoType"
  '((:UNSPECIFIED_TRAJECTORY_INFORMATION . 0)
    (:JOINT_ACCELERATION_LIMIT_REACHED . 1)
    (:JOINT_SPEED_LIMIT_REACHED . 2)
    (:JOINT_POSITION_LIMIT_REACHED . 3)
    (:JOINT_TORQUE_LIMIT_REACHED . 4)
    (:SINGULARITY_REGION . 5)
    (:INVERSE_KINEMATIC_FAILED . 6)
    (:CARTESIAN_ACCELERATION_LIMIT_REACHED . 7)
    (:CARTESIAN_SPEED_LIMIT_REACHED . 8)
    (:CARTESIAN_POSITION_LIMIT_REACHED . 9)
    (:CARTESIAN_WRENCH_LIMIT_REACHED . 10)
    (:ENTERING_PROTECTION_ZONE . 11)
    (:WAYPOINT_REACHED . 12)
    (:TRAJECTORY_OK . 13))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryInfoType>) ostream)
  "Serializes a message object of type '<TrajectoryInfoType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryInfoType>) istream)
  "Deserializes a message object of type '<TrajectoryInfoType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryInfoType>)))
  "Returns string type for a message object of type '<TrajectoryInfoType>"
  "kortex_driver/TrajectoryInfoType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryInfoType)))
  "Returns string type for a message object of type 'TrajectoryInfoType"
  "kortex_driver/TrajectoryInfoType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryInfoType>)))
  "Returns md5sum for a message object of type '<TrajectoryInfoType>"
  "8bf652b45448cd88f4d8d2fc90a3634d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryInfoType)))
  "Returns md5sum for a message object of type 'TrajectoryInfoType"
  "8bf652b45448cd88f4d8d2fc90a3634d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryInfoType>)))
  "Returns full string definition for message of type '<TrajectoryInfoType>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_TRAJECTORY_INFORMATION = 0~%~%uint32 JOINT_ACCELERATION_LIMIT_REACHED = 1~%~%uint32 JOINT_SPEED_LIMIT_REACHED = 2~%~%uint32 JOINT_POSITION_LIMIT_REACHED = 3~%~%uint32 JOINT_TORQUE_LIMIT_REACHED = 4~%~%uint32 SINGULARITY_REGION = 5~%~%uint32 INVERSE_KINEMATIC_FAILED = 6~%~%uint32 CARTESIAN_ACCELERATION_LIMIT_REACHED = 7~%~%uint32 CARTESIAN_SPEED_LIMIT_REACHED = 8~%~%uint32 CARTESIAN_POSITION_LIMIT_REACHED = 9~%~%uint32 CARTESIAN_WRENCH_LIMIT_REACHED = 10~%~%uint32 ENTERING_PROTECTION_ZONE = 11~%~%uint32 WAYPOINT_REACHED = 12~%~%uint32 TRAJECTORY_OK = 13~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryInfoType)))
  "Returns full string definition for message of type 'TrajectoryInfoType"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_TRAJECTORY_INFORMATION = 0~%~%uint32 JOINT_ACCELERATION_LIMIT_REACHED = 1~%~%uint32 JOINT_SPEED_LIMIT_REACHED = 2~%~%uint32 JOINT_POSITION_LIMIT_REACHED = 3~%~%uint32 JOINT_TORQUE_LIMIT_REACHED = 4~%~%uint32 SINGULARITY_REGION = 5~%~%uint32 INVERSE_KINEMATIC_FAILED = 6~%~%uint32 CARTESIAN_ACCELERATION_LIMIT_REACHED = 7~%~%uint32 CARTESIAN_SPEED_LIMIT_REACHED = 8~%~%uint32 CARTESIAN_POSITION_LIMIT_REACHED = 9~%~%uint32 CARTESIAN_WRENCH_LIMIT_REACHED = 10~%~%uint32 ENTERING_PROTECTION_ZONE = 11~%~%uint32 WAYPOINT_REACHED = 12~%~%uint32 TRAJECTORY_OK = 13~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryInfoType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryInfoType>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryInfoType
))
