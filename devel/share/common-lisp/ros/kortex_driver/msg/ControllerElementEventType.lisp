; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControllerElementEventType.msg.html

(cl:defclass <ControllerElementEventType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ControllerElementEventType (<ControllerElementEventType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerElementEventType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerElementEventType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControllerElementEventType> is deprecated: use kortex_driver-msg:ControllerElementEventType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ControllerElementEventType>)))
    "Constants for message type '<ControllerElementEventType>"
  '((:UNSPECIFIED_CONTROLLER_ELEMENT_EVENT . 0)
    (:AXIS_MOVED . 1)
    (:BUTTON_DOWN . 2)
    (:BUTTON_UP . 3)
    (:BUTTON_CLICK . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ControllerElementEventType)))
    "Constants for message type 'ControllerElementEventType"
  '((:UNSPECIFIED_CONTROLLER_ELEMENT_EVENT . 0)
    (:AXIS_MOVED . 1)
    (:BUTTON_DOWN . 2)
    (:BUTTON_UP . 3)
    (:BUTTON_CLICK . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerElementEventType>) ostream)
  "Serializes a message object of type '<ControllerElementEventType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerElementEventType>) istream)
  "Deserializes a message object of type '<ControllerElementEventType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerElementEventType>)))
  "Returns string type for a message object of type '<ControllerElementEventType>"
  "kortex_driver/ControllerElementEventType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerElementEventType)))
  "Returns string type for a message object of type 'ControllerElementEventType"
  "kortex_driver/ControllerElementEventType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerElementEventType>)))
  "Returns md5sum for a message object of type '<ControllerElementEventType>"
  "30410419f52cbfa760b386b2213e4309")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerElementEventType)))
  "Returns md5sum for a message object of type 'ControllerElementEventType"
  "30410419f52cbfa760b386b2213e4309")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerElementEventType>)))
  "Returns full string definition for message of type '<ControllerElementEventType>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CONTROLLER_ELEMENT_EVENT = 0~%~%uint32 AXIS_MOVED = 1~%~%uint32 BUTTON_DOWN = 2~%~%uint32 BUTTON_UP = 3~%~%uint32 BUTTON_CLICK = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerElementEventType)))
  "Returns full string definition for message of type 'ControllerElementEventType"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CONTROLLER_ELEMENT_EVENT = 0~%~%uint32 AXIS_MOVED = 1~%~%uint32 BUTTON_DOWN = 2~%~%uint32 BUTTON_UP = 3~%~%uint32 BUTTON_CLICK = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerElementEventType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerElementEventType>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerElementEventType
))
