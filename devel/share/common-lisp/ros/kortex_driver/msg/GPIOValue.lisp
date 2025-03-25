; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude GPIOValue.msg.html

(cl:defclass <GPIOValue> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GPIOValue (<GPIOValue>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GPIOValue>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GPIOValue)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<GPIOValue> is deprecated: use kortex_driver-msg:GPIOValue instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GPIOValue>)))
    "Constants for message type '<GPIOValue>"
  '((:GPIO_VALUE_UNSPECIFIED . 0)
    (:GPIO_VALUE_LOW . 1)
    (:GPIO_VALUE_HIGH . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GPIOValue)))
    "Constants for message type 'GPIOValue"
  '((:GPIO_VALUE_UNSPECIFIED . 0)
    (:GPIO_VALUE_LOW . 1)
    (:GPIO_VALUE_HIGH . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GPIOValue>) ostream)
  "Serializes a message object of type '<GPIOValue>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GPIOValue>) istream)
  "Deserializes a message object of type '<GPIOValue>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GPIOValue>)))
  "Returns string type for a message object of type '<GPIOValue>"
  "kortex_driver/GPIOValue")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GPIOValue)))
  "Returns string type for a message object of type 'GPIOValue"
  "kortex_driver/GPIOValue")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GPIOValue>)))
  "Returns md5sum for a message object of type '<GPIOValue>"
  "2450504e1dcf956a73a1061ef8b94f94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GPIOValue)))
  "Returns md5sum for a message object of type 'GPIOValue"
  "2450504e1dcf956a73a1061ef8b94f94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GPIOValue>)))
  "Returns full string definition for message of type '<GPIOValue>"
  (cl:format cl:nil "~%uint32 GPIO_VALUE_UNSPECIFIED = 0~%~%uint32 GPIO_VALUE_LOW = 1~%~%uint32 GPIO_VALUE_HIGH = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GPIOValue)))
  "Returns full string definition for message of type 'GPIOValue"
  (cl:format cl:nil "~%uint32 GPIO_VALUE_UNSPECIFIED = 0~%~%uint32 GPIO_VALUE_LOW = 1~%~%uint32 GPIO_VALUE_HIGH = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GPIOValue>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GPIOValue>))
  "Converts a ROS message object to a list"
  (cl:list 'GPIOValue
))
