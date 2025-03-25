; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude BridgeType.msg.html

(cl:defclass <BridgeType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass BridgeType (<BridgeType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BridgeType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BridgeType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<BridgeType> is deprecated: use kortex_driver-msg:BridgeType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<BridgeType>)))
    "Constants for message type '<BridgeType>"
  '((:BRIDGE_TYPE_UNSPECIFIED . 0)
    (:BRIDGE_TYPE_UART . 1)
    (:BRIDGE_TYPE_TELNET . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'BridgeType)))
    "Constants for message type 'BridgeType"
  '((:BRIDGE_TYPE_UNSPECIFIED . 0)
    (:BRIDGE_TYPE_UART . 1)
    (:BRIDGE_TYPE_TELNET . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BridgeType>) ostream)
  "Serializes a message object of type '<BridgeType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BridgeType>) istream)
  "Deserializes a message object of type '<BridgeType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BridgeType>)))
  "Returns string type for a message object of type '<BridgeType>"
  "kortex_driver/BridgeType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BridgeType)))
  "Returns string type for a message object of type 'BridgeType"
  "kortex_driver/BridgeType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BridgeType>)))
  "Returns md5sum for a message object of type '<BridgeType>"
  "b8e85d50bf3b32805a4cb6f83f1b2f4f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BridgeType)))
  "Returns md5sum for a message object of type 'BridgeType"
  "b8e85d50bf3b32805a4cb6f83f1b2f4f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BridgeType>)))
  "Returns full string definition for message of type '<BridgeType>"
  (cl:format cl:nil "~%uint32 BRIDGE_TYPE_UNSPECIFIED = 0~%~%uint32 BRIDGE_TYPE_UART = 1~%~%uint32 BRIDGE_TYPE_TELNET = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BridgeType)))
  "Returns full string definition for message of type 'BridgeType"
  (cl:format cl:nil "~%uint32 BRIDGE_TYPE_UNSPECIFIED = 0~%~%uint32 BRIDGE_TYPE_UART = 1~%~%uint32 BRIDGE_TYPE_TELNET = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BridgeType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BridgeType>))
  "Converts a ROS message object to a list"
  (cl:list 'BridgeType
))
