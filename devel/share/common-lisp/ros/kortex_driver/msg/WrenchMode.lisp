; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude WrenchMode.msg.html

(cl:defclass <WrenchMode> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass WrenchMode (<WrenchMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WrenchMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WrenchMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<WrenchMode> is deprecated: use kortex_driver-msg:WrenchMode instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<WrenchMode>)))
    "Constants for message type '<WrenchMode>"
  '((:UNSPECIFIED_WRENCH_MODE . 0)
    (:WRENCH_RESTRICTED . 1)
    (:WRENCH_NORMAL . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'WrenchMode)))
    "Constants for message type 'WrenchMode"
  '((:UNSPECIFIED_WRENCH_MODE . 0)
    (:WRENCH_RESTRICTED . 1)
    (:WRENCH_NORMAL . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WrenchMode>) ostream)
  "Serializes a message object of type '<WrenchMode>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WrenchMode>) istream)
  "Deserializes a message object of type '<WrenchMode>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WrenchMode>)))
  "Returns string type for a message object of type '<WrenchMode>"
  "kortex_driver/WrenchMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WrenchMode)))
  "Returns string type for a message object of type 'WrenchMode"
  "kortex_driver/WrenchMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WrenchMode>)))
  "Returns md5sum for a message object of type '<WrenchMode>"
  "21f4098a76f681f49eb3f39d12684c72")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WrenchMode)))
  "Returns md5sum for a message object of type 'WrenchMode"
  "21f4098a76f681f49eb3f39d12684c72")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WrenchMode>)))
  "Returns full string definition for message of type '<WrenchMode>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_WRENCH_MODE = 0~%~%uint32 WRENCH_RESTRICTED = 1~%~%uint32 WRENCH_NORMAL = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WrenchMode)))
  "Returns full string definition for message of type 'WrenchMode"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_WRENCH_MODE = 0~%~%uint32 WRENCH_RESTRICTED = 1~%~%uint32 WRENCH_NORMAL = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WrenchMode>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WrenchMode>))
  "Converts a ROS message object to a list"
  (cl:list 'WrenchMode
))
