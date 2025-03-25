; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude InterfaceModuleType.msg.html

(cl:defclass <InterfaceModuleType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass InterfaceModuleType (<InterfaceModuleType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InterfaceModuleType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InterfaceModuleType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<InterfaceModuleType> is deprecated: use kortex_driver-msg:InterfaceModuleType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<InterfaceModuleType>)))
    "Constants for message type '<InterfaceModuleType>"
  '((:INTERFACE_MODULE_TYPE_UNSPECIFIED . 0)
    (:INTERFACE_MODULE_TYPE_NOT_INSTALLED . 1)
    (:INTERFACE_MODULE_TYPE_L53 . 2)
    (:INTERFACE_MODULE_TYPE_L31 . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'InterfaceModuleType)))
    "Constants for message type 'InterfaceModuleType"
  '((:INTERFACE_MODULE_TYPE_UNSPECIFIED . 0)
    (:INTERFACE_MODULE_TYPE_NOT_INSTALLED . 1)
    (:INTERFACE_MODULE_TYPE_L53 . 2)
    (:INTERFACE_MODULE_TYPE_L31 . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InterfaceModuleType>) ostream)
  "Serializes a message object of type '<InterfaceModuleType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InterfaceModuleType>) istream)
  "Deserializes a message object of type '<InterfaceModuleType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InterfaceModuleType>)))
  "Returns string type for a message object of type '<InterfaceModuleType>"
  "kortex_driver/InterfaceModuleType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InterfaceModuleType)))
  "Returns string type for a message object of type 'InterfaceModuleType"
  "kortex_driver/InterfaceModuleType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InterfaceModuleType>)))
  "Returns md5sum for a message object of type '<InterfaceModuleType>"
  "f61dc75115f701d8cd0d9b3951030045")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InterfaceModuleType)))
  "Returns md5sum for a message object of type 'InterfaceModuleType"
  "f61dc75115f701d8cd0d9b3951030045")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InterfaceModuleType>)))
  "Returns full string definition for message of type '<InterfaceModuleType>"
  (cl:format cl:nil "~%uint32 INTERFACE_MODULE_TYPE_UNSPECIFIED = 0~%~%uint32 INTERFACE_MODULE_TYPE_NOT_INSTALLED = 1~%~%uint32 INTERFACE_MODULE_TYPE_L53 = 2~%~%uint32 INTERFACE_MODULE_TYPE_L31 = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InterfaceModuleType)))
  "Returns full string definition for message of type 'InterfaceModuleType"
  (cl:format cl:nil "~%uint32 INTERFACE_MODULE_TYPE_UNSPECIFIED = 0~%~%uint32 INTERFACE_MODULE_TYPE_NOT_INSTALLED = 1~%~%uint32 INTERFACE_MODULE_TYPE_L53 = 2~%~%uint32 INTERFACE_MODULE_TYPE_L31 = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InterfaceModuleType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InterfaceModuleType>))
  "Converts a ROS message object to a list"
  (cl:list 'InterfaceModuleType
))
