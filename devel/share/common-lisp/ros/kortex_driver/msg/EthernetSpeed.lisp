; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude EthernetSpeed.msg.html

(cl:defclass <EthernetSpeed> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass EthernetSpeed (<EthernetSpeed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EthernetSpeed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EthernetSpeed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<EthernetSpeed> is deprecated: use kortex_driver-msg:EthernetSpeed instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<EthernetSpeed>)))
    "Constants for message type '<EthernetSpeed>"
  '((:ETHERNET_SPEED_UNSPECIFIED . 0)
    (:ETHERNET_SPEED_10M . 1)
    (:ETHERNET_SPEED_100M . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'EthernetSpeed)))
    "Constants for message type 'EthernetSpeed"
  '((:ETHERNET_SPEED_UNSPECIFIED . 0)
    (:ETHERNET_SPEED_10M . 1)
    (:ETHERNET_SPEED_100M . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EthernetSpeed>) ostream)
  "Serializes a message object of type '<EthernetSpeed>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EthernetSpeed>) istream)
  "Deserializes a message object of type '<EthernetSpeed>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EthernetSpeed>)))
  "Returns string type for a message object of type '<EthernetSpeed>"
  "kortex_driver/EthernetSpeed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EthernetSpeed)))
  "Returns string type for a message object of type 'EthernetSpeed"
  "kortex_driver/EthernetSpeed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EthernetSpeed>)))
  "Returns md5sum for a message object of type '<EthernetSpeed>"
  "03143ee942f3291a850b685ab265f926")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EthernetSpeed)))
  "Returns md5sum for a message object of type 'EthernetSpeed"
  "03143ee942f3291a850b685ab265f926")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EthernetSpeed>)))
  "Returns full string definition for message of type '<EthernetSpeed>"
  (cl:format cl:nil "~%uint32 ETHERNET_SPEED_UNSPECIFIED = 0~%~%uint32 ETHERNET_SPEED_10M = 1~%~%uint32 ETHERNET_SPEED_100M = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EthernetSpeed)))
  "Returns full string definition for message of type 'EthernetSpeed"
  (cl:format cl:nil "~%uint32 ETHERNET_SPEED_UNSPECIFIED = 0~%~%uint32 ETHERNET_SPEED_10M = 1~%~%uint32 ETHERNET_SPEED_100M = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EthernetSpeed>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EthernetSpeed>))
  "Converts a ROS message object to a list"
  (cl:list 'EthernetSpeed
))
