; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SafetyStatusValue.msg.html

(cl:defclass <SafetyStatusValue> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SafetyStatusValue (<SafetyStatusValue>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SafetyStatusValue>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SafetyStatusValue)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SafetyStatusValue> is deprecated: use kortex_driver-msg:SafetyStatusValue instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SafetyStatusValue>)))
    "Constants for message type '<SafetyStatusValue>"
  '((:SAFETY_STATUS_UNSPECIFIED . 0)
    (:SAFETY_STATUS_WARNING . 1)
    (:SAFETY_STATUS_ERROR . 2)
    (:SAFETY_STATUS_NORMAL . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SafetyStatusValue)))
    "Constants for message type 'SafetyStatusValue"
  '((:SAFETY_STATUS_UNSPECIFIED . 0)
    (:SAFETY_STATUS_WARNING . 1)
    (:SAFETY_STATUS_ERROR . 2)
    (:SAFETY_STATUS_NORMAL . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SafetyStatusValue>) ostream)
  "Serializes a message object of type '<SafetyStatusValue>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SafetyStatusValue>) istream)
  "Deserializes a message object of type '<SafetyStatusValue>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SafetyStatusValue>)))
  "Returns string type for a message object of type '<SafetyStatusValue>"
  "kortex_driver/SafetyStatusValue")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SafetyStatusValue)))
  "Returns string type for a message object of type 'SafetyStatusValue"
  "kortex_driver/SafetyStatusValue")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SafetyStatusValue>)))
  "Returns md5sum for a message object of type '<SafetyStatusValue>"
  "6f4f91637eeaf53ea8343202803cf904")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SafetyStatusValue)))
  "Returns md5sum for a message object of type 'SafetyStatusValue"
  "6f4f91637eeaf53ea8343202803cf904")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SafetyStatusValue>)))
  "Returns full string definition for message of type '<SafetyStatusValue>"
  (cl:format cl:nil "~%uint32 SAFETY_STATUS_UNSPECIFIED = 0~%~%uint32 SAFETY_STATUS_WARNING = 1~%~%uint32 SAFETY_STATUS_ERROR = 2~%~%uint32 SAFETY_STATUS_NORMAL = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SafetyStatusValue)))
  "Returns full string definition for message of type 'SafetyStatusValue"
  (cl:format cl:nil "~%uint32 SAFETY_STATUS_UNSPECIFIED = 0~%~%uint32 SAFETY_STATUS_WARNING = 1~%~%uint32 SAFETY_STATUS_ERROR = 2~%~%uint32 SAFETY_STATUS_NORMAL = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SafetyStatusValue>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SafetyStatusValue>))
  "Converts a ROS message object to a list"
  (cl:list 'SafetyStatusValue
))
