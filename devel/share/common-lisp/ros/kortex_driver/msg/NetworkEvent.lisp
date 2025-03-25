; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude NetworkEvent.msg.html

(cl:defclass <NetworkEvent> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass NetworkEvent (<NetworkEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NetworkEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NetworkEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<NetworkEvent> is deprecated: use kortex_driver-msg:NetworkEvent instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<NetworkEvent>)))
    "Constants for message type '<NetworkEvent>"
  '((:UNSPECIFIED_NETWORK_EVENT . 0)
    (:WIFI_CONNECTED . 1)
    (:WIFI_DISCONNECTED . 2)
    (:WIFI_SCAN_STARTED . 3)
    (:WIFI_SCAN_RESULTS . 4)
    (:WIFI_SCAN_FAILED . 5)
    (:WIFI_NOT_FOUND . 6)
    (:WIFI_ASSOC_REJECTED . 7)
    (:WIFI_AUTH_WRONG_KEY . 8)
    (:WIFI_AUTH_CONN_FAILED . 9)
    (:WIFI_AUTH_FAILED . 10)
    (:WIFI_ENABLED . 11)
    (:WIFI_DISABLED . 12)
    (:BLUETOOTH_ENABLED . 13)
    (:BLUETOOTH_DISABLED . 14))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'NetworkEvent)))
    "Constants for message type 'NetworkEvent"
  '((:UNSPECIFIED_NETWORK_EVENT . 0)
    (:WIFI_CONNECTED . 1)
    (:WIFI_DISCONNECTED . 2)
    (:WIFI_SCAN_STARTED . 3)
    (:WIFI_SCAN_RESULTS . 4)
    (:WIFI_SCAN_FAILED . 5)
    (:WIFI_NOT_FOUND . 6)
    (:WIFI_ASSOC_REJECTED . 7)
    (:WIFI_AUTH_WRONG_KEY . 8)
    (:WIFI_AUTH_CONN_FAILED . 9)
    (:WIFI_AUTH_FAILED . 10)
    (:WIFI_ENABLED . 11)
    (:WIFI_DISABLED . 12)
    (:BLUETOOTH_ENABLED . 13)
    (:BLUETOOTH_DISABLED . 14))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NetworkEvent>) ostream)
  "Serializes a message object of type '<NetworkEvent>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NetworkEvent>) istream)
  "Deserializes a message object of type '<NetworkEvent>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NetworkEvent>)))
  "Returns string type for a message object of type '<NetworkEvent>"
  "kortex_driver/NetworkEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NetworkEvent)))
  "Returns string type for a message object of type 'NetworkEvent"
  "kortex_driver/NetworkEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NetworkEvent>)))
  "Returns md5sum for a message object of type '<NetworkEvent>"
  "342adcc6d6071ab5d5d0b6d3b3ff3e84")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NetworkEvent)))
  "Returns md5sum for a message object of type 'NetworkEvent"
  "342adcc6d6071ab5d5d0b6d3b3ff3e84")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NetworkEvent>)))
  "Returns full string definition for message of type '<NetworkEvent>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_NETWORK_EVENT = 0~%~%uint32 WIFI_CONNECTED = 1~%~%uint32 WIFI_DISCONNECTED = 2~%~%uint32 WIFI_SCAN_STARTED = 3~%~%uint32 WIFI_SCAN_RESULTS = 4~%~%uint32 WIFI_SCAN_FAILED = 5~%~%uint32 WIFI_NOT_FOUND = 6~%~%uint32 WIFI_ASSOC_REJECTED = 7~%~%uint32 WIFI_AUTH_WRONG_KEY = 8~%~%uint32 WIFI_AUTH_CONN_FAILED = 9~%~%uint32 WIFI_AUTH_FAILED = 10~%~%uint32 WIFI_ENABLED = 11~%~%uint32 WIFI_DISABLED = 12~%~%uint32 BLUETOOTH_ENABLED = 13~%~%uint32 BLUETOOTH_DISABLED = 14~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NetworkEvent)))
  "Returns full string definition for message of type 'NetworkEvent"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_NETWORK_EVENT = 0~%~%uint32 WIFI_CONNECTED = 1~%~%uint32 WIFI_DISCONNECTED = 2~%~%uint32 WIFI_SCAN_STARTED = 3~%~%uint32 WIFI_SCAN_RESULTS = 4~%~%uint32 WIFI_SCAN_FAILED = 5~%~%uint32 WIFI_NOT_FOUND = 6~%~%uint32 WIFI_ASSOC_REJECTED = 7~%~%uint32 WIFI_AUTH_WRONG_KEY = 8~%~%uint32 WIFI_AUTH_CONN_FAILED = 9~%~%uint32 WIFI_AUTH_FAILED = 10~%~%uint32 WIFI_ENABLED = 11~%~%uint32 WIFI_DISABLED = 12~%~%uint32 BLUETOOTH_ENABLED = 13~%~%uint32 BLUETOOTH_DISABLED = 14~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NetworkEvent>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NetworkEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'NetworkEvent
))
