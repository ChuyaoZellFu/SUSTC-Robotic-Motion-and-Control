; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Base_ServiceVersion.msg.html

(cl:defclass <Base_ServiceVersion> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Base_ServiceVersion (<Base_ServiceVersion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Base_ServiceVersion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Base_ServiceVersion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Base_ServiceVersion> is deprecated: use kortex_driver-msg:Base_ServiceVersion instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Base_ServiceVersion>)))
    "Constants for message type '<Base_ServiceVersion>"
  '((:RESERVED_0 . 0)
    (:CURRENT_VERSION . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Base_ServiceVersion)))
    "Constants for message type 'Base_ServiceVersion"
  '((:RESERVED_0 . 0)
    (:CURRENT_VERSION . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Base_ServiceVersion>) ostream)
  "Serializes a message object of type '<Base_ServiceVersion>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Base_ServiceVersion>) istream)
  "Deserializes a message object of type '<Base_ServiceVersion>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Base_ServiceVersion>)))
  "Returns string type for a message object of type '<Base_ServiceVersion>"
  "kortex_driver/Base_ServiceVersion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_ServiceVersion)))
  "Returns string type for a message object of type 'Base_ServiceVersion"
  "kortex_driver/Base_ServiceVersion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Base_ServiceVersion>)))
  "Returns md5sum for a message object of type '<Base_ServiceVersion>"
  "d8624be8e7830df48d9964b7612fa5b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Base_ServiceVersion)))
  "Returns md5sum for a message object of type 'Base_ServiceVersion"
  "d8624be8e7830df48d9964b7612fa5b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Base_ServiceVersion>)))
  "Returns full string definition for message of type '<Base_ServiceVersion>"
  (cl:format cl:nil "~%uint32 RESERVED_0 = 0~%~%uint32 CURRENT_VERSION = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Base_ServiceVersion)))
  "Returns full string definition for message of type 'Base_ServiceVersion"
  (cl:format cl:nil "~%uint32 RESERVED_0 = 0~%~%uint32 CURRENT_VERSION = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Base_ServiceVersion>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Base_ServiceVersion>))
  "Converts a ROS message object to a list"
  (cl:list 'Base_ServiceVersion
))
