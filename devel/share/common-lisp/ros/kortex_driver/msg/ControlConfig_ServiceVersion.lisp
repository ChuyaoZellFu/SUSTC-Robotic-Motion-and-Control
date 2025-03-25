; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControlConfig_ServiceVersion.msg.html

(cl:defclass <ControlConfig_ServiceVersion> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ControlConfig_ServiceVersion (<ControlConfig_ServiceVersion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlConfig_ServiceVersion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlConfig_ServiceVersion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControlConfig_ServiceVersion> is deprecated: use kortex_driver-msg:ControlConfig_ServiceVersion instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ControlConfig_ServiceVersion>)))
    "Constants for message type '<ControlConfig_ServiceVersion>"
  '((:RESERVED_0 . 0)
    (:CURRENT_VERSION . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ControlConfig_ServiceVersion)))
    "Constants for message type 'ControlConfig_ServiceVersion"
  '((:RESERVED_0 . 0)
    (:CURRENT_VERSION . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlConfig_ServiceVersion>) ostream)
  "Serializes a message object of type '<ControlConfig_ServiceVersion>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlConfig_ServiceVersion>) istream)
  "Deserializes a message object of type '<ControlConfig_ServiceVersion>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlConfig_ServiceVersion>)))
  "Returns string type for a message object of type '<ControlConfig_ServiceVersion>"
  "kortex_driver/ControlConfig_ServiceVersion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlConfig_ServiceVersion)))
  "Returns string type for a message object of type 'ControlConfig_ServiceVersion"
  "kortex_driver/ControlConfig_ServiceVersion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlConfig_ServiceVersion>)))
  "Returns md5sum for a message object of type '<ControlConfig_ServiceVersion>"
  "d8624be8e7830df48d9964b7612fa5b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlConfig_ServiceVersion)))
  "Returns md5sum for a message object of type 'ControlConfig_ServiceVersion"
  "d8624be8e7830df48d9964b7612fa5b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlConfig_ServiceVersion>)))
  "Returns full string definition for message of type '<ControlConfig_ServiceVersion>"
  (cl:format cl:nil "~%uint32 RESERVED_0 = 0~%~%uint32 CURRENT_VERSION = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlConfig_ServiceVersion)))
  "Returns full string definition for message of type 'ControlConfig_ServiceVersion"
  (cl:format cl:nil "~%uint32 RESERVED_0 = 0~%~%uint32 CURRENT_VERSION = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlConfig_ServiceVersion>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlConfig_ServiceVersion>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlConfig_ServiceVersion
))
