; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude DeviceManager_ServiceVersion.msg.html

(cl:defclass <DeviceManager_ServiceVersion> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass DeviceManager_ServiceVersion (<DeviceManager_ServiceVersion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeviceManager_ServiceVersion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeviceManager_ServiceVersion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<DeviceManager_ServiceVersion> is deprecated: use kortex_driver-msg:DeviceManager_ServiceVersion instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<DeviceManager_ServiceVersion>)))
    "Constants for message type '<DeviceManager_ServiceVersion>"
  '((:RESERVED_0 . 0)
    (:CURRENT_VERSION . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'DeviceManager_ServiceVersion)))
    "Constants for message type 'DeviceManager_ServiceVersion"
  '((:RESERVED_0 . 0)
    (:CURRENT_VERSION . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeviceManager_ServiceVersion>) ostream)
  "Serializes a message object of type '<DeviceManager_ServiceVersion>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeviceManager_ServiceVersion>) istream)
  "Deserializes a message object of type '<DeviceManager_ServiceVersion>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeviceManager_ServiceVersion>)))
  "Returns string type for a message object of type '<DeviceManager_ServiceVersion>"
  "kortex_driver/DeviceManager_ServiceVersion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeviceManager_ServiceVersion)))
  "Returns string type for a message object of type 'DeviceManager_ServiceVersion"
  "kortex_driver/DeviceManager_ServiceVersion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeviceManager_ServiceVersion>)))
  "Returns md5sum for a message object of type '<DeviceManager_ServiceVersion>"
  "d8624be8e7830df48d9964b7612fa5b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeviceManager_ServiceVersion)))
  "Returns md5sum for a message object of type 'DeviceManager_ServiceVersion"
  "d8624be8e7830df48d9964b7612fa5b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeviceManager_ServiceVersion>)))
  "Returns full string definition for message of type '<DeviceManager_ServiceVersion>"
  (cl:format cl:nil "~%uint32 RESERVED_0 = 0~%~%uint32 CURRENT_VERSION = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeviceManager_ServiceVersion)))
  "Returns full string definition for message of type 'DeviceManager_ServiceVersion"
  (cl:format cl:nil "~%uint32 RESERVED_0 = 0~%~%uint32 CURRENT_VERSION = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeviceManager_ServiceVersion>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeviceManager_ServiceVersion>))
  "Converts a ROS message object to a list"
  (cl:list 'DeviceManager_ServiceVersion
))
