; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControllerBehavior.msg.html

(cl:defclass <ControllerBehavior> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ControllerBehavior (<ControllerBehavior>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerBehavior>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerBehavior)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControllerBehavior> is deprecated: use kortex_driver-msg:ControllerBehavior instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ControllerBehavior>)))
    "Constants for message type '<ControllerBehavior>"
  '((:UNSPECIFIED_CONTROLLER_BEHAVIOR . 0)
    (:CONTROLLER_BUTTON_DOWN . 1)
    (:CONTROLLER_BUTTON_UP . 2)
    (:CONTROLLER_AXIS_POSITIVE . 3)
    (:CONTROLLER_AXIS_NEGATIVE . 4)
    (:CONTROLLER_BUTTON_CLICK . 5))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ControllerBehavior)))
    "Constants for message type 'ControllerBehavior"
  '((:UNSPECIFIED_CONTROLLER_BEHAVIOR . 0)
    (:CONTROLLER_BUTTON_DOWN . 1)
    (:CONTROLLER_BUTTON_UP . 2)
    (:CONTROLLER_AXIS_POSITIVE . 3)
    (:CONTROLLER_AXIS_NEGATIVE . 4)
    (:CONTROLLER_BUTTON_CLICK . 5))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerBehavior>) ostream)
  "Serializes a message object of type '<ControllerBehavior>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerBehavior>) istream)
  "Deserializes a message object of type '<ControllerBehavior>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerBehavior>)))
  "Returns string type for a message object of type '<ControllerBehavior>"
  "kortex_driver/ControllerBehavior")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerBehavior)))
  "Returns string type for a message object of type 'ControllerBehavior"
  "kortex_driver/ControllerBehavior")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerBehavior>)))
  "Returns md5sum for a message object of type '<ControllerBehavior>"
  "12553caa8500dc92a4c4cfda3f89fc6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerBehavior)))
  "Returns md5sum for a message object of type 'ControllerBehavior"
  "12553caa8500dc92a4c4cfda3f89fc6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerBehavior>)))
  "Returns full string definition for message of type '<ControllerBehavior>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CONTROLLER_BEHAVIOR = 0~%~%uint32 CONTROLLER_BUTTON_DOWN = 1~%~%uint32 CONTROLLER_BUTTON_UP = 2~%~%uint32 CONTROLLER_AXIS_POSITIVE = 3~%~%uint32 CONTROLLER_AXIS_NEGATIVE = 4~%~%uint32 CONTROLLER_BUTTON_CLICK = 5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerBehavior)))
  "Returns full string definition for message of type 'ControllerBehavior"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CONTROLLER_BEHAVIOR = 0~%~%uint32 CONTROLLER_BUTTON_DOWN = 1~%~%uint32 CONTROLLER_BUTTON_UP = 2~%~%uint32 CONTROLLER_AXIS_POSITIVE = 3~%~%uint32 CONTROLLER_AXIS_NEGATIVE = 4~%~%uint32 CONTROLLER_BUTTON_CLICK = 5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerBehavior>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerBehavior>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerBehavior
))
