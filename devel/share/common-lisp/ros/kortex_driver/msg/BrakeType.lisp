; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude BrakeType.msg.html

(cl:defclass <BrakeType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass BrakeType (<BrakeType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BrakeType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BrakeType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<BrakeType> is deprecated: use kortex_driver-msg:BrakeType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<BrakeType>)))
    "Constants for message type '<BrakeType>"
  '((:BRAKE_TYPE_UNSPECIFIED . 0)
    (:BRAKE_TYPE_NOT_INSTALLED . 1)
    (:BRAKE_TYPE_SPOKE . 2)
    (:BRAKE_TYPE_CLUTCH . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'BrakeType)))
    "Constants for message type 'BrakeType"
  '((:BRAKE_TYPE_UNSPECIFIED . 0)
    (:BRAKE_TYPE_NOT_INSTALLED . 1)
    (:BRAKE_TYPE_SPOKE . 2)
    (:BRAKE_TYPE_CLUTCH . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BrakeType>) ostream)
  "Serializes a message object of type '<BrakeType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BrakeType>) istream)
  "Deserializes a message object of type '<BrakeType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BrakeType>)))
  "Returns string type for a message object of type '<BrakeType>"
  "kortex_driver/BrakeType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BrakeType)))
  "Returns string type for a message object of type 'BrakeType"
  "kortex_driver/BrakeType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BrakeType>)))
  "Returns md5sum for a message object of type '<BrakeType>"
  "856901ae6854740e9adfa01cae483501")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BrakeType)))
  "Returns md5sum for a message object of type 'BrakeType"
  "856901ae6854740e9adfa01cae483501")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BrakeType>)))
  "Returns full string definition for message of type '<BrakeType>"
  (cl:format cl:nil "~%uint32 BRAKE_TYPE_UNSPECIFIED = 0~%~%uint32 BRAKE_TYPE_NOT_INSTALLED = 1~%~%uint32 BRAKE_TYPE_SPOKE = 2~%~%uint32 BRAKE_TYPE_CLUTCH = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BrakeType)))
  "Returns full string definition for message of type 'BrakeType"
  (cl:format cl:nil "~%uint32 BRAKE_TYPE_UNSPECIFIED = 0~%~%uint32 BRAKE_TYPE_NOT_INSTALLED = 1~%~%uint32 BRAKE_TYPE_SPOKE = 2~%~%uint32 BRAKE_TYPE_CLUTCH = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BrakeType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BrakeType>))
  "Converts a ROS message object to a list"
  (cl:list 'BrakeType
))
