; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude RobotEvent.msg.html

(cl:defclass <RobotEvent> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass RobotEvent (<RobotEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<RobotEvent> is deprecated: use kortex_driver-msg:RobotEvent instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<RobotEvent>)))
    "Constants for message type '<RobotEvent>"
  '((:UNSPECIFIED_ROBOT_EVENT . 0)
    (:ARM_CONNECTED . 1)
    (:ARM_DISCONNECTED . 2)
    (:TOOL_CONNECTED . 5)
    (:TOOL_DISCONNECTED . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'RobotEvent)))
    "Constants for message type 'RobotEvent"
  '((:UNSPECIFIED_ROBOT_EVENT . 0)
    (:ARM_CONNECTED . 1)
    (:ARM_DISCONNECTED . 2)
    (:TOOL_CONNECTED . 5)
    (:TOOL_DISCONNECTED . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotEvent>) ostream)
  "Serializes a message object of type '<RobotEvent>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotEvent>) istream)
  "Deserializes a message object of type '<RobotEvent>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotEvent>)))
  "Returns string type for a message object of type '<RobotEvent>"
  "kortex_driver/RobotEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotEvent)))
  "Returns string type for a message object of type 'RobotEvent"
  "kortex_driver/RobotEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotEvent>)))
  "Returns md5sum for a message object of type '<RobotEvent>"
  "7edc4e787111520cc97803b60a159bc5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotEvent)))
  "Returns md5sum for a message object of type 'RobotEvent"
  "7edc4e787111520cc97803b60a159bc5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotEvent>)))
  "Returns full string definition for message of type '<RobotEvent>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_ROBOT_EVENT = 0~%~%uint32 ARM_CONNECTED = 1~%~%uint32 ARM_DISCONNECTED = 2~%~%uint32 TOOL_CONNECTED = 5~%~%uint32 TOOL_DISCONNECTED = 6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotEvent)))
  "Returns full string definition for message of type 'RobotEvent"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_ROBOT_EVENT = 0~%~%uint32 ARM_CONNECTED = 1~%~%uint32 ARM_DISCONNECTED = 2~%~%uint32 TOOL_CONNECTED = 5~%~%uint32 TOOL_DISCONNECTED = 6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotEvent>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotEvent
))
