; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude GpioBehavior.msg.html

(cl:defclass <GpioBehavior> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GpioBehavior (<GpioBehavior>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GpioBehavior>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GpioBehavior)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<GpioBehavior> is deprecated: use kortex_driver-msg:GpioBehavior instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GpioBehavior>)))
    "Constants for message type '<GpioBehavior>"
  '((:UNSPECIFIED_GPIO_BEHAVIOR . 0)
    (:GPIO_FALLING . 1)
    (:GPIO_RISING . 2)
    (:GPIO_PULSE_LOW . 3)
    (:GPIO_PULSE_HIGH . 4)
    (:GPIO_ANALOG_CHANGE . 5))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GpioBehavior)))
    "Constants for message type 'GpioBehavior"
  '((:UNSPECIFIED_GPIO_BEHAVIOR . 0)
    (:GPIO_FALLING . 1)
    (:GPIO_RISING . 2)
    (:GPIO_PULSE_LOW . 3)
    (:GPIO_PULSE_HIGH . 4)
    (:GPIO_ANALOG_CHANGE . 5))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GpioBehavior>) ostream)
  "Serializes a message object of type '<GpioBehavior>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GpioBehavior>) istream)
  "Deserializes a message object of type '<GpioBehavior>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GpioBehavior>)))
  "Returns string type for a message object of type '<GpioBehavior>"
  "kortex_driver/GpioBehavior")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GpioBehavior)))
  "Returns string type for a message object of type 'GpioBehavior"
  "kortex_driver/GpioBehavior")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GpioBehavior>)))
  "Returns md5sum for a message object of type '<GpioBehavior>"
  "a285ed144abb1d2ba02c8fa81ee67296")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GpioBehavior)))
  "Returns md5sum for a message object of type 'GpioBehavior"
  "a285ed144abb1d2ba02c8fa81ee67296")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GpioBehavior>)))
  "Returns full string definition for message of type '<GpioBehavior>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_GPIO_BEHAVIOR = 0~%~%uint32 GPIO_FALLING = 1~%~%uint32 GPIO_RISING = 2~%~%uint32 GPIO_PULSE_LOW = 3~%~%uint32 GPIO_PULSE_HIGH = 4~%~%uint32 GPIO_ANALOG_CHANGE = 5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GpioBehavior)))
  "Returns full string definition for message of type 'GpioBehavior"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_GPIO_BEHAVIOR = 0~%~%uint32 GPIO_FALLING = 1~%~%uint32 GPIO_RISING = 2~%~%uint32 GPIO_PULSE_LOW = 3~%~%uint32 GPIO_PULSE_HIGH = 4~%~%uint32 GPIO_ANALOG_CHANGE = 5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GpioBehavior>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GpioBehavior>))
  "Converts a ROS message object to a list"
  (cl:list 'GpioBehavior
))
