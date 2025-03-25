; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ArmState.msg.html

(cl:defclass <ArmState> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ArmState (<ArmState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ArmState> is deprecated: use kortex_driver-msg:ArmState instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ArmState>)))
    "Constants for message type '<ArmState>"
  '((:ARMSTATE_UNSPECIFIED . 0)
    (:ARMSTATE_BASE_INITIALIZATION . 1)
    (:ARMSTATE_IDLE . 2)
    (:ARMSTATE_INITIALIZATION . 3)
    (:ARMSTATE_IN_FAULT . 4)
    (:ARMSTATE_MAINTENANCE . 5)
    (:ARMSTATE_SERVOING_LOW_LEVEL . 6)
    (:ARMSTATE_SERVOING_READY . 7)
    (:ARMSTATE_SERVOING_PLAYING_SEQUENCE . 8)
    (:ARMSTATE_SERVOING_MANUALLY_CONTROLLED . 9)
    (:ARMSTATE_BRAKE_RELEASING . 10)
    (:ARMSTATE_RESERVED . 255))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ArmState)))
    "Constants for message type 'ArmState"
  '((:ARMSTATE_UNSPECIFIED . 0)
    (:ARMSTATE_BASE_INITIALIZATION . 1)
    (:ARMSTATE_IDLE . 2)
    (:ARMSTATE_INITIALIZATION . 3)
    (:ARMSTATE_IN_FAULT . 4)
    (:ARMSTATE_MAINTENANCE . 5)
    (:ARMSTATE_SERVOING_LOW_LEVEL . 6)
    (:ARMSTATE_SERVOING_READY . 7)
    (:ARMSTATE_SERVOING_PLAYING_SEQUENCE . 8)
    (:ARMSTATE_SERVOING_MANUALLY_CONTROLLED . 9)
    (:ARMSTATE_BRAKE_RELEASING . 10)
    (:ARMSTATE_RESERVED . 255))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmState>) ostream)
  "Serializes a message object of type '<ArmState>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmState>) istream)
  "Deserializes a message object of type '<ArmState>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmState>)))
  "Returns string type for a message object of type '<ArmState>"
  "kortex_driver/ArmState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmState)))
  "Returns string type for a message object of type 'ArmState"
  "kortex_driver/ArmState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmState>)))
  "Returns md5sum for a message object of type '<ArmState>"
  "ac681740837f166518d3ae6a4e6eeda1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmState)))
  "Returns md5sum for a message object of type 'ArmState"
  "ac681740837f166518d3ae6a4e6eeda1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmState>)))
  "Returns full string definition for message of type '<ArmState>"
  (cl:format cl:nil "~%uint32 ARMSTATE_UNSPECIFIED = 0~%~%uint32 ARMSTATE_BASE_INITIALIZATION = 1~%~%uint32 ARMSTATE_IDLE = 2~%~%uint32 ARMSTATE_INITIALIZATION = 3~%~%uint32 ARMSTATE_IN_FAULT = 4~%~%uint32 ARMSTATE_MAINTENANCE = 5~%~%uint32 ARMSTATE_SERVOING_LOW_LEVEL = 6~%~%uint32 ARMSTATE_SERVOING_READY = 7~%~%uint32 ARMSTATE_SERVOING_PLAYING_SEQUENCE = 8~%~%uint32 ARMSTATE_SERVOING_MANUALLY_CONTROLLED = 9~%~%uint32 ARMSTATE_BRAKE_RELEASING = 10~%~%uint32 ARMSTATE_RESERVED = 255~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmState)))
  "Returns full string definition for message of type 'ArmState"
  (cl:format cl:nil "~%uint32 ARMSTATE_UNSPECIFIED = 0~%~%uint32 ARMSTATE_BASE_INITIALIZATION = 1~%~%uint32 ARMSTATE_IDLE = 2~%~%uint32 ARMSTATE_INITIALIZATION = 3~%~%uint32 ARMSTATE_IN_FAULT = 4~%~%uint32 ARMSTATE_MAINTENANCE = 5~%~%uint32 ARMSTATE_SERVOING_LOW_LEVEL = 6~%~%uint32 ARMSTATE_SERVOING_READY = 7~%~%uint32 ARMSTATE_SERVOING_PLAYING_SEQUENCE = 8~%~%uint32 ARMSTATE_SERVOING_MANUALLY_CONTROLLED = 9~%~%uint32 ARMSTATE_BRAKE_RELEASING = 10~%~%uint32 ARMSTATE_RESERVED = 255~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmState>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmState>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmState
))
