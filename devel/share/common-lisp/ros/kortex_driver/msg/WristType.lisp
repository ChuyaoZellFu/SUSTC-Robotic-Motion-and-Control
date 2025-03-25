; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude WristType.msg.html

(cl:defclass <WristType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass WristType (<WristType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WristType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WristType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<WristType> is deprecated: use kortex_driver-msg:WristType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<WristType>)))
    "Constants for message type '<WristType>"
  '((:WRIST_TYPE_UNSPECIFIED . 0)
    (:WRIST_TYPE_NOT_APPLICABLE . 1)
    (:WRIST_TYPE_SPHERICAL . 2)
    (:WRIST_TYPE_CURVED . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'WristType)))
    "Constants for message type 'WristType"
  '((:WRIST_TYPE_UNSPECIFIED . 0)
    (:WRIST_TYPE_NOT_APPLICABLE . 1)
    (:WRIST_TYPE_SPHERICAL . 2)
    (:WRIST_TYPE_CURVED . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WristType>) ostream)
  "Serializes a message object of type '<WristType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WristType>) istream)
  "Deserializes a message object of type '<WristType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WristType>)))
  "Returns string type for a message object of type '<WristType>"
  "kortex_driver/WristType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WristType)))
  "Returns string type for a message object of type 'WristType"
  "kortex_driver/WristType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WristType>)))
  "Returns md5sum for a message object of type '<WristType>"
  "dce76d4e649d6e568f93c37bf10d6238")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WristType)))
  "Returns md5sum for a message object of type 'WristType"
  "dce76d4e649d6e568f93c37bf10d6238")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WristType>)))
  "Returns full string definition for message of type '<WristType>"
  (cl:format cl:nil "~%uint32 WRIST_TYPE_UNSPECIFIED = 0~%~%uint32 WRIST_TYPE_NOT_APPLICABLE = 1~%~%uint32 WRIST_TYPE_SPHERICAL = 2~%~%uint32 WRIST_TYPE_CURVED = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WristType)))
  "Returns full string definition for message of type 'WristType"
  (cl:format cl:nil "~%uint32 WRIST_TYPE_UNSPECIFIED = 0~%~%uint32 WRIST_TYPE_NOT_APPLICABLE = 1~%~%uint32 WRIST_TYPE_SPHERICAL = 2~%~%uint32 WRIST_TYPE_CURVED = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WristType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WristType>))
  "Converts a ROS message object to a list"
  (cl:list 'WristType
))
