; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude VisionConfig_ServiceVersion.msg.html

(cl:defclass <VisionConfig_ServiceVersion> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass VisionConfig_ServiceVersion (<VisionConfig_ServiceVersion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisionConfig_ServiceVersion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisionConfig_ServiceVersion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<VisionConfig_ServiceVersion> is deprecated: use kortex_driver-msg:VisionConfig_ServiceVersion instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<VisionConfig_ServiceVersion>)))
    "Constants for message type '<VisionConfig_ServiceVersion>"
  '((:RESERVED_0 . 0)
    (:CURRENT_VERSION . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'VisionConfig_ServiceVersion)))
    "Constants for message type 'VisionConfig_ServiceVersion"
  '((:RESERVED_0 . 0)
    (:CURRENT_VERSION . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisionConfig_ServiceVersion>) ostream)
  "Serializes a message object of type '<VisionConfig_ServiceVersion>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisionConfig_ServiceVersion>) istream)
  "Deserializes a message object of type '<VisionConfig_ServiceVersion>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisionConfig_ServiceVersion>)))
  "Returns string type for a message object of type '<VisionConfig_ServiceVersion>"
  "kortex_driver/VisionConfig_ServiceVersion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisionConfig_ServiceVersion)))
  "Returns string type for a message object of type 'VisionConfig_ServiceVersion"
  "kortex_driver/VisionConfig_ServiceVersion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisionConfig_ServiceVersion>)))
  "Returns md5sum for a message object of type '<VisionConfig_ServiceVersion>"
  "d8624be8e7830df48d9964b7612fa5b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisionConfig_ServiceVersion)))
  "Returns md5sum for a message object of type 'VisionConfig_ServiceVersion"
  "d8624be8e7830df48d9964b7612fa5b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisionConfig_ServiceVersion>)))
  "Returns full string definition for message of type '<VisionConfig_ServiceVersion>"
  (cl:format cl:nil "~%uint32 RESERVED_0 = 0~%~%uint32 CURRENT_VERSION = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisionConfig_ServiceVersion)))
  "Returns full string definition for message of type 'VisionConfig_ServiceVersion"
  (cl:format cl:nil "~%uint32 RESERVED_0 = 0~%~%uint32 CURRENT_VERSION = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisionConfig_ServiceVersion>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisionConfig_ServiceVersion>))
  "Converts a ROS message object to a list"
  (cl:list 'VisionConfig_ServiceVersion
))
