; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude GPIOPull.msg.html

(cl:defclass <GPIOPull> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GPIOPull (<GPIOPull>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GPIOPull>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GPIOPull)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<GPIOPull> is deprecated: use kortex_driver-msg:GPIOPull instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GPIOPull>)))
    "Constants for message type '<GPIOPull>"
  '((:GPIO_PULL_UNSPECIFIED . 0)
    (:GPIO_PULL_NONE . 1)
    (:GPIO_PULL_UP . 2)
    (:GPIO_PULL_DOWN . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GPIOPull)))
    "Constants for message type 'GPIOPull"
  '((:GPIO_PULL_UNSPECIFIED . 0)
    (:GPIO_PULL_NONE . 1)
    (:GPIO_PULL_UP . 2)
    (:GPIO_PULL_DOWN . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GPIOPull>) ostream)
  "Serializes a message object of type '<GPIOPull>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GPIOPull>) istream)
  "Deserializes a message object of type '<GPIOPull>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GPIOPull>)))
  "Returns string type for a message object of type '<GPIOPull>"
  "kortex_driver/GPIOPull")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GPIOPull)))
  "Returns string type for a message object of type 'GPIOPull"
  "kortex_driver/GPIOPull")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GPIOPull>)))
  "Returns md5sum for a message object of type '<GPIOPull>"
  "76d74f4a9f22d9530bd83becdc56fba5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GPIOPull)))
  "Returns md5sum for a message object of type 'GPIOPull"
  "76d74f4a9f22d9530bd83becdc56fba5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GPIOPull>)))
  "Returns full string definition for message of type '<GPIOPull>"
  (cl:format cl:nil "~%uint32 GPIO_PULL_UNSPECIFIED = 0~%~%uint32 GPIO_PULL_NONE = 1~%~%uint32 GPIO_PULL_UP = 2~%~%uint32 GPIO_PULL_DOWN = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GPIOPull)))
  "Returns full string definition for message of type 'GPIOPull"
  (cl:format cl:nil "~%uint32 GPIO_PULL_UNSPECIFIED = 0~%~%uint32 GPIO_PULL_NONE = 1~%~%uint32 GPIO_PULL_UP = 2~%~%uint32 GPIO_PULL_DOWN = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GPIOPull>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GPIOPull>))
  "Converts a ROS message object to a list"
  (cl:list 'GPIOPull
))
