; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude EthernetDuplex.msg.html

(cl:defclass <EthernetDuplex> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass EthernetDuplex (<EthernetDuplex>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EthernetDuplex>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EthernetDuplex)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<EthernetDuplex> is deprecated: use kortex_driver-msg:EthernetDuplex instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<EthernetDuplex>)))
    "Constants for message type '<EthernetDuplex>"
  '((:ETHERNET_DUPLEX_UNSPECIFIED . 0)
    (:ETHERNET_DUPLEX_HALF . 1)
    (:ETHERNET_DUPLEX_FULL . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'EthernetDuplex)))
    "Constants for message type 'EthernetDuplex"
  '((:ETHERNET_DUPLEX_UNSPECIFIED . 0)
    (:ETHERNET_DUPLEX_HALF . 1)
    (:ETHERNET_DUPLEX_FULL . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EthernetDuplex>) ostream)
  "Serializes a message object of type '<EthernetDuplex>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EthernetDuplex>) istream)
  "Deserializes a message object of type '<EthernetDuplex>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EthernetDuplex>)))
  "Returns string type for a message object of type '<EthernetDuplex>"
  "kortex_driver/EthernetDuplex")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EthernetDuplex)))
  "Returns string type for a message object of type 'EthernetDuplex"
  "kortex_driver/EthernetDuplex")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EthernetDuplex>)))
  "Returns md5sum for a message object of type '<EthernetDuplex>"
  "cd452927d77476fb60f0cc73e416a7ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EthernetDuplex)))
  "Returns md5sum for a message object of type 'EthernetDuplex"
  "cd452927d77476fb60f0cc73e416a7ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EthernetDuplex>)))
  "Returns full string definition for message of type '<EthernetDuplex>"
  (cl:format cl:nil "~%uint32 ETHERNET_DUPLEX_UNSPECIFIED = 0~%~%uint32 ETHERNET_DUPLEX_HALF = 1~%~%uint32 ETHERNET_DUPLEX_FULL = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EthernetDuplex)))
  "Returns full string definition for message of type 'EthernetDuplex"
  (cl:format cl:nil "~%uint32 ETHERNET_DUPLEX_UNSPECIFIED = 0~%~%uint32 ETHERNET_DUPLEX_HALF = 1~%~%uint32 ETHERNET_DUPLEX_FULL = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EthernetDuplex>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EthernetDuplex>))
  "Converts a ROS message object to a list"
  (cl:list 'EthernetDuplex
))
