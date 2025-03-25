; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControllerType.msg.html

(cl:defclass <ControllerType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ControllerType (<ControllerType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControllerType> is deprecated: use kortex_driver-msg:ControllerType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ControllerType>)))
    "Constants for message type '<ControllerType>"
  '((:UNSPECIFIED_CONTROLLER_TYPE . 0)
    (:XBOX_CONTROLLER . 1)
    (:WRIST_CONTROLLER . 2)
    (:BASIC_JOYSTICK_CONTROLLER . 3)
    (:BASE_GPIO_CONTROLLER . 4)
    (:GPIO_JOYSTICK . 5))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ControllerType)))
    "Constants for message type 'ControllerType"
  '((:UNSPECIFIED_CONTROLLER_TYPE . 0)
    (:XBOX_CONTROLLER . 1)
    (:WRIST_CONTROLLER . 2)
    (:BASIC_JOYSTICK_CONTROLLER . 3)
    (:BASE_GPIO_CONTROLLER . 4)
    (:GPIO_JOYSTICK . 5))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerType>) ostream)
  "Serializes a message object of type '<ControllerType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerType>) istream)
  "Deserializes a message object of type '<ControllerType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerType>)))
  "Returns string type for a message object of type '<ControllerType>"
  "kortex_driver/ControllerType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerType)))
  "Returns string type for a message object of type 'ControllerType"
  "kortex_driver/ControllerType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerType>)))
  "Returns md5sum for a message object of type '<ControllerType>"
  "482367554f194246d15dc5fb1ede90ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerType)))
  "Returns md5sum for a message object of type 'ControllerType"
  "482367554f194246d15dc5fb1ede90ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerType>)))
  "Returns full string definition for message of type '<ControllerType>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CONTROLLER_TYPE = 0~%~%uint32 XBOX_CONTROLLER = 1~%~%uint32 WRIST_CONTROLLER = 2~%~%uint32 BASIC_JOYSTICK_CONTROLLER = 3~%~%uint32 BASE_GPIO_CONTROLLER = 4~%~%uint32 GPIO_JOYSTICK = 5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerType)))
  "Returns full string definition for message of type 'ControllerType"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CONTROLLER_TYPE = 0~%~%uint32 XBOX_CONTROLLER = 1~%~%uint32 WRIST_CONTROLLER = 2~%~%uint32 BASIC_JOYSTICK_CONTROLLER = 3~%~%uint32 BASE_GPIO_CONTROLLER = 4~%~%uint32 GPIO_JOYSTICK = 5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerType>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerType
))
