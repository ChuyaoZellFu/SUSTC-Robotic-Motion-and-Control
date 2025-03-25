; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude BridgeStatus.msg.html

(cl:defclass <BridgeStatus> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass BridgeStatus (<BridgeStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BridgeStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BridgeStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<BridgeStatus> is deprecated: use kortex_driver-msg:BridgeStatus instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<BridgeStatus>)))
    "Constants for message type '<BridgeStatus>"
  '((:BRIDGE_STATUS_OK . 0)
    (:BRIDGE_STATUS_OUTP_UNAVAILABLE . 1)
    (:BRIDGE_STATUS_UNKNOWN_DEVID . 2)
    (:BRIDGE_STATUS_UNKNOWN_BRIDGE_TYPE . 3)
    (:BRIDGE_STATUS_NOT_FOUND . 4)
    (:BRIDGE_STATUS_NOT_INITIALIZED . 5)
    (:BRIDGE_STATUS_UNKNOWN . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'BridgeStatus)))
    "Constants for message type 'BridgeStatus"
  '((:BRIDGE_STATUS_OK . 0)
    (:BRIDGE_STATUS_OUTP_UNAVAILABLE . 1)
    (:BRIDGE_STATUS_UNKNOWN_DEVID . 2)
    (:BRIDGE_STATUS_UNKNOWN_BRIDGE_TYPE . 3)
    (:BRIDGE_STATUS_NOT_FOUND . 4)
    (:BRIDGE_STATUS_NOT_INITIALIZED . 5)
    (:BRIDGE_STATUS_UNKNOWN . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BridgeStatus>) ostream)
  "Serializes a message object of type '<BridgeStatus>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BridgeStatus>) istream)
  "Deserializes a message object of type '<BridgeStatus>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BridgeStatus>)))
  "Returns string type for a message object of type '<BridgeStatus>"
  "kortex_driver/BridgeStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BridgeStatus)))
  "Returns string type for a message object of type 'BridgeStatus"
  "kortex_driver/BridgeStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BridgeStatus>)))
  "Returns md5sum for a message object of type '<BridgeStatus>"
  "9c6586b899b39a3f8a12bc04700166a0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BridgeStatus)))
  "Returns md5sum for a message object of type 'BridgeStatus"
  "9c6586b899b39a3f8a12bc04700166a0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BridgeStatus>)))
  "Returns full string definition for message of type '<BridgeStatus>"
  (cl:format cl:nil "~%uint32 BRIDGE_STATUS_OK = 0~%~%uint32 BRIDGE_STATUS_OUTP_UNAVAILABLE = 1~%~%uint32 BRIDGE_STATUS_UNKNOWN_DEVID = 2~%~%uint32 BRIDGE_STATUS_UNKNOWN_BRIDGE_TYPE = 3~%~%uint32 BRIDGE_STATUS_NOT_FOUND = 4~%~%uint32 BRIDGE_STATUS_NOT_INITIALIZED = 5~%~%uint32 BRIDGE_STATUS_UNKNOWN = 6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BridgeStatus)))
  "Returns full string definition for message of type 'BridgeStatus"
  (cl:format cl:nil "~%uint32 BRIDGE_STATUS_OK = 0~%~%uint32 BRIDGE_STATUS_OUTP_UNAVAILABLE = 1~%~%uint32 BRIDGE_STATUS_UNKNOWN_DEVID = 2~%~%uint32 BRIDGE_STATUS_UNKNOWN_BRIDGE_TYPE = 3~%~%uint32 BRIDGE_STATUS_NOT_FOUND = 4~%~%uint32 BRIDGE_STATUS_NOT_INITIALIZED = 5~%~%uint32 BRIDGE_STATUS_UNKNOWN = 6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BridgeStatus>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BridgeStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'BridgeStatus
))
