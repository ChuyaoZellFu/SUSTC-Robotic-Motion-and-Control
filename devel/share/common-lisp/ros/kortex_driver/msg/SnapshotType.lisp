; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SnapshotType.msg.html

(cl:defclass <SnapshotType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SnapshotType (<SnapshotType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SnapshotType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SnapshotType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SnapshotType> is deprecated: use kortex_driver-msg:SnapshotType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SnapshotType>)))
    "Constants for message type '<SnapshotType>"
  '((:UNSPECIFIED_SNAPSHOT . 0)
    (:CARTESIAN_POSITION_SNAPSHOT . 1)
    (:JOINT_POSITION_SNAPSHOT . 2)
    (:GRIPPER_SNAPSHOT . 3)
    (:COMBINED_SNAPSHOT . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SnapshotType)))
    "Constants for message type 'SnapshotType"
  '((:UNSPECIFIED_SNAPSHOT . 0)
    (:CARTESIAN_POSITION_SNAPSHOT . 1)
    (:JOINT_POSITION_SNAPSHOT . 2)
    (:GRIPPER_SNAPSHOT . 3)
    (:COMBINED_SNAPSHOT . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SnapshotType>) ostream)
  "Serializes a message object of type '<SnapshotType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SnapshotType>) istream)
  "Deserializes a message object of type '<SnapshotType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SnapshotType>)))
  "Returns string type for a message object of type '<SnapshotType>"
  "kortex_driver/SnapshotType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SnapshotType)))
  "Returns string type for a message object of type 'SnapshotType"
  "kortex_driver/SnapshotType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SnapshotType>)))
  "Returns md5sum for a message object of type '<SnapshotType>"
  "a9efee68393bf09e7b23d3b042a31552")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SnapshotType)))
  "Returns md5sum for a message object of type 'SnapshotType"
  "a9efee68393bf09e7b23d3b042a31552")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SnapshotType>)))
  "Returns full string definition for message of type '<SnapshotType>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_SNAPSHOT = 0~%~%uint32 CARTESIAN_POSITION_SNAPSHOT = 1~%~%uint32 JOINT_POSITION_SNAPSHOT = 2~%~%uint32 GRIPPER_SNAPSHOT = 3~%~%uint32 COMBINED_SNAPSHOT = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SnapshotType)))
  "Returns full string definition for message of type 'SnapshotType"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_SNAPSHOT = 0~%~%uint32 CARTESIAN_POSITION_SNAPSHOT = 1~%~%uint32 JOINT_POSITION_SNAPSHOT = 2~%~%uint32 GRIPPER_SNAPSHOT = 3~%~%uint32 COMBINED_SNAPSHOT = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SnapshotType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SnapshotType>))
  "Converts a ROS message object to a list"
  (cl:list 'SnapshotType
))
