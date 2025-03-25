; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude EthernetDevice.msg.html

(cl:defclass <EthernetDevice> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass EthernetDevice (<EthernetDevice>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EthernetDevice>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EthernetDevice)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<EthernetDevice> is deprecated: use kortex_driver-msg:EthernetDevice instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<EthernetDevice>)))
    "Constants for message type '<EthernetDevice>"
  '((:ETHERNET_DEVICE_UNSPECIFIED . 0)
    (:ETHERNET_DEVICE_EXPANSION . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'EthernetDevice)))
    "Constants for message type 'EthernetDevice"
  '((:ETHERNET_DEVICE_UNSPECIFIED . 0)
    (:ETHERNET_DEVICE_EXPANSION . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EthernetDevice>) ostream)
  "Serializes a message object of type '<EthernetDevice>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EthernetDevice>) istream)
  "Deserializes a message object of type '<EthernetDevice>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EthernetDevice>)))
  "Returns string type for a message object of type '<EthernetDevice>"
  "kortex_driver/EthernetDevice")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EthernetDevice)))
  "Returns string type for a message object of type 'EthernetDevice"
  "kortex_driver/EthernetDevice")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EthernetDevice>)))
  "Returns md5sum for a message object of type '<EthernetDevice>"
  "cbad10c4e89ab17bbf73ffd6e85d4b44")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EthernetDevice)))
  "Returns md5sum for a message object of type 'EthernetDevice"
  "cbad10c4e89ab17bbf73ffd6e85d4b44")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EthernetDevice>)))
  "Returns full string definition for message of type '<EthernetDevice>"
  (cl:format cl:nil "~%uint32 ETHERNET_DEVICE_UNSPECIFIED = 0~%~%uint32 ETHERNET_DEVICE_EXPANSION = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EthernetDevice)))
  "Returns full string definition for message of type 'EthernetDevice"
  (cl:format cl:nil "~%uint32 ETHERNET_DEVICE_UNSPECIFIED = 0~%~%uint32 ETHERNET_DEVICE_EXPANSION = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EthernetDevice>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EthernetDevice>))
  "Converts a ROS message object to a list"
  (cl:list 'EthernetDevice
))
