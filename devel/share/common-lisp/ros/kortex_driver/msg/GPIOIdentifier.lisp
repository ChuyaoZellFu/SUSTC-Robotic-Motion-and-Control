; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude GPIOIdentifier.msg.html

(cl:defclass <GPIOIdentifier> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GPIOIdentifier (<GPIOIdentifier>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GPIOIdentifier>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GPIOIdentifier)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<GPIOIdentifier> is deprecated: use kortex_driver-msg:GPIOIdentifier instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GPIOIdentifier>)))
    "Constants for message type '<GPIOIdentifier>"
  '((:GPIO_IDENTIFIER_UNSPECIFIED . 0)
    (:GPIO_IDENTIFIER_1 . 1)
    (:GPIO_IDENTIFIER_2 . 2)
    (:GPIO_IDENTIFIER_3 . 3)
    (:GPIO_IDENTIFIER_4 . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GPIOIdentifier)))
    "Constants for message type 'GPIOIdentifier"
  '((:GPIO_IDENTIFIER_UNSPECIFIED . 0)
    (:GPIO_IDENTIFIER_1 . 1)
    (:GPIO_IDENTIFIER_2 . 2)
    (:GPIO_IDENTIFIER_3 . 3)
    (:GPIO_IDENTIFIER_4 . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GPIOIdentifier>) ostream)
  "Serializes a message object of type '<GPIOIdentifier>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GPIOIdentifier>) istream)
  "Deserializes a message object of type '<GPIOIdentifier>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GPIOIdentifier>)))
  "Returns string type for a message object of type '<GPIOIdentifier>"
  "kortex_driver/GPIOIdentifier")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GPIOIdentifier)))
  "Returns string type for a message object of type 'GPIOIdentifier"
  "kortex_driver/GPIOIdentifier")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GPIOIdentifier>)))
  "Returns md5sum for a message object of type '<GPIOIdentifier>"
  "6d6dda494f04fdbc115c7ef311e5d45d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GPIOIdentifier)))
  "Returns md5sum for a message object of type 'GPIOIdentifier"
  "6d6dda494f04fdbc115c7ef311e5d45d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GPIOIdentifier>)))
  "Returns full string definition for message of type '<GPIOIdentifier>"
  (cl:format cl:nil "~%uint32 GPIO_IDENTIFIER_UNSPECIFIED = 0~%~%uint32 GPIO_IDENTIFIER_1 = 1~%~%uint32 GPIO_IDENTIFIER_2 = 2~%~%uint32 GPIO_IDENTIFIER_3 = 3~%~%uint32 GPIO_IDENTIFIER_4 = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GPIOIdentifier)))
  "Returns full string definition for message of type 'GPIOIdentifier"
  (cl:format cl:nil "~%uint32 GPIO_IDENTIFIER_UNSPECIFIED = 0~%~%uint32 GPIO_IDENTIFIER_1 = 1~%~%uint32 GPIO_IDENTIFIER_2 = 2~%~%uint32 GPIO_IDENTIFIER_3 = 3~%~%uint32 GPIO_IDENTIFIER_4 = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GPIOIdentifier>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GPIOIdentifier>))
  "Converts a ROS message object to a list"
  (cl:list 'GPIOIdentifier
))
