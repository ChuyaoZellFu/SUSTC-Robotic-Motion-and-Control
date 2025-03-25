; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude NetworkType.msg.html

(cl:defclass <NetworkType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass NetworkType (<NetworkType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NetworkType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NetworkType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<NetworkType> is deprecated: use kortex_driver-msg:NetworkType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<NetworkType>)))
    "Constants for message type '<NetworkType>"
  '((:UNSPECIFIED_NETWORK_TYPE . 0)
    (:WIFI . 1)
    (:WIRED_ETHERNET . 2)
    (:WIRED_MICROUSB . 3)
    (:WIRED_USB . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'NetworkType)))
    "Constants for message type 'NetworkType"
  '((:UNSPECIFIED_NETWORK_TYPE . 0)
    (:WIFI . 1)
    (:WIRED_ETHERNET . 2)
    (:WIRED_MICROUSB . 3)
    (:WIRED_USB . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NetworkType>) ostream)
  "Serializes a message object of type '<NetworkType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NetworkType>) istream)
  "Deserializes a message object of type '<NetworkType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NetworkType>)))
  "Returns string type for a message object of type '<NetworkType>"
  "kortex_driver/NetworkType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NetworkType)))
  "Returns string type for a message object of type 'NetworkType"
  "kortex_driver/NetworkType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NetworkType>)))
  "Returns md5sum for a message object of type '<NetworkType>"
  "de77e3c574da819850bde657b3223236")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NetworkType)))
  "Returns md5sum for a message object of type 'NetworkType"
  "de77e3c574da819850bde657b3223236")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NetworkType>)))
  "Returns full string definition for message of type '<NetworkType>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_NETWORK_TYPE = 0~%~%uint32 WIFI = 1~%~%uint32 WIRED_ETHERNET = 2~%~%uint32 WIRED_MICROUSB = 3~%~%uint32 WIRED_USB = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NetworkType)))
  "Returns full string definition for message of type 'NetworkType"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_NETWORK_TYPE = 0~%~%uint32 WIFI = 1~%~%uint32 WIRED_ETHERNET = 2~%~%uint32 WIRED_MICROUSB = 3~%~%uint32 WIRED_USB = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NetworkType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NetworkType>))
  "Converts a ROS message object to a list"
  (cl:list 'NetworkType
))
