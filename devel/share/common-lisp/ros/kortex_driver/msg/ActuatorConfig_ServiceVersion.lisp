; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ActuatorConfig_ServiceVersion.msg.html

(cl:defclass <ActuatorConfig_ServiceVersion> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ActuatorConfig_ServiceVersion (<ActuatorConfig_ServiceVersion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActuatorConfig_ServiceVersion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActuatorConfig_ServiceVersion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ActuatorConfig_ServiceVersion> is deprecated: use kortex_driver-msg:ActuatorConfig_ServiceVersion instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ActuatorConfig_ServiceVersion>)))
    "Constants for message type '<ActuatorConfig_ServiceVersion>"
  '((:RESERVED_0 . 0)
    (:CURRENT_VERSION . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ActuatorConfig_ServiceVersion)))
    "Constants for message type 'ActuatorConfig_ServiceVersion"
  '((:RESERVED_0 . 0)
    (:CURRENT_VERSION . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActuatorConfig_ServiceVersion>) ostream)
  "Serializes a message object of type '<ActuatorConfig_ServiceVersion>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActuatorConfig_ServiceVersion>) istream)
  "Deserializes a message object of type '<ActuatorConfig_ServiceVersion>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActuatorConfig_ServiceVersion>)))
  "Returns string type for a message object of type '<ActuatorConfig_ServiceVersion>"
  "kortex_driver/ActuatorConfig_ServiceVersion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActuatorConfig_ServiceVersion)))
  "Returns string type for a message object of type 'ActuatorConfig_ServiceVersion"
  "kortex_driver/ActuatorConfig_ServiceVersion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActuatorConfig_ServiceVersion>)))
  "Returns md5sum for a message object of type '<ActuatorConfig_ServiceVersion>"
  "d8624be8e7830df48d9964b7612fa5b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActuatorConfig_ServiceVersion)))
  "Returns md5sum for a message object of type 'ActuatorConfig_ServiceVersion"
  "d8624be8e7830df48d9964b7612fa5b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActuatorConfig_ServiceVersion>)))
  "Returns full string definition for message of type '<ActuatorConfig_ServiceVersion>"
  (cl:format cl:nil "~%uint32 RESERVED_0 = 0~%~%uint32 CURRENT_VERSION = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActuatorConfig_ServiceVersion)))
  "Returns full string definition for message of type 'ActuatorConfig_ServiceVersion"
  (cl:format cl:nil "~%uint32 RESERVED_0 = 0~%~%uint32 CURRENT_VERSION = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActuatorConfig_ServiceVersion>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActuatorConfig_ServiceVersion>))
  "Converts a ROS message object to a list"
  (cl:list 'ActuatorConfig_ServiceVersion
))
