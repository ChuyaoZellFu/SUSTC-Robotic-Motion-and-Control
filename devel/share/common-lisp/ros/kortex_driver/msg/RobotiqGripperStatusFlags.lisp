; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude RobotiqGripperStatusFlags.msg.html

(cl:defclass <RobotiqGripperStatusFlags> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass RobotiqGripperStatusFlags (<RobotiqGripperStatusFlags>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotiqGripperStatusFlags>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotiqGripperStatusFlags)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<RobotiqGripperStatusFlags> is deprecated: use kortex_driver-msg:RobotiqGripperStatusFlags instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<RobotiqGripperStatusFlags>)))
    "Constants for message type '<RobotiqGripperStatusFlags>"
  '((:UNSPECIFIED_ROBOTIQ_STATUS . 0)
    (:ROBOTIQ_STAT_INITIALIZED . 1)
    (:ROBOTIQ_STAT_OBJECT_DETECTED . 2)
    (:ROBOTIQ_STAT_POS_REACHED . 4)
    (:ROBOTIQ_STAT_STOPPED . 8))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'RobotiqGripperStatusFlags)))
    "Constants for message type 'RobotiqGripperStatusFlags"
  '((:UNSPECIFIED_ROBOTIQ_STATUS . 0)
    (:ROBOTIQ_STAT_INITIALIZED . 1)
    (:ROBOTIQ_STAT_OBJECT_DETECTED . 2)
    (:ROBOTIQ_STAT_POS_REACHED . 4)
    (:ROBOTIQ_STAT_STOPPED . 8))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotiqGripperStatusFlags>) ostream)
  "Serializes a message object of type '<RobotiqGripperStatusFlags>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotiqGripperStatusFlags>) istream)
  "Deserializes a message object of type '<RobotiqGripperStatusFlags>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotiqGripperStatusFlags>)))
  "Returns string type for a message object of type '<RobotiqGripperStatusFlags>"
  "kortex_driver/RobotiqGripperStatusFlags")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotiqGripperStatusFlags)))
  "Returns string type for a message object of type 'RobotiqGripperStatusFlags"
  "kortex_driver/RobotiqGripperStatusFlags")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotiqGripperStatusFlags>)))
  "Returns md5sum for a message object of type '<RobotiqGripperStatusFlags>"
  "289d3fd6bd510e8e9380318bbf77ab70")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotiqGripperStatusFlags)))
  "Returns md5sum for a message object of type 'RobotiqGripperStatusFlags"
  "289d3fd6bd510e8e9380318bbf77ab70")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotiqGripperStatusFlags>)))
  "Returns full string definition for message of type '<RobotiqGripperStatusFlags>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_ROBOTIQ_STATUS = 0~%~%uint32 ROBOTIQ_STAT_INITIALIZED = 1~%~%uint32 ROBOTIQ_STAT_OBJECT_DETECTED = 2~%~%uint32 ROBOTIQ_STAT_POS_REACHED = 4~%~%uint32 ROBOTIQ_STAT_STOPPED = 8~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotiqGripperStatusFlags)))
  "Returns full string definition for message of type 'RobotiqGripperStatusFlags"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_ROBOTIQ_STATUS = 0~%~%uint32 ROBOTIQ_STAT_INITIALIZED = 1~%~%uint32 ROBOTIQ_STAT_OBJECT_DETECTED = 2~%~%uint32 ROBOTIQ_STAT_POS_REACHED = 4~%~%uint32 ROBOTIQ_STAT_STOPPED = 8~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotiqGripperStatusFlags>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotiqGripperStatusFlags>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotiqGripperStatusFlags
))
