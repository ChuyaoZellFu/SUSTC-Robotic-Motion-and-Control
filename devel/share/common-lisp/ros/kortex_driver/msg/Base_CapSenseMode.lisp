; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Base_CapSenseMode.msg.html

(cl:defclass <Base_CapSenseMode> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Base_CapSenseMode (<Base_CapSenseMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Base_CapSenseMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Base_CapSenseMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Base_CapSenseMode> is deprecated: use kortex_driver-msg:Base_CapSenseMode instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Base_CapSenseMode>)))
    "Constants for message type '<Base_CapSenseMode>"
  '((:RESERVED . 0)
    (:INACTIVE . 1)
    (:ACTIVE_AUTO_THRESHOLD . 2)
    (:ACTIVE_NOISE_ATT . 4)
    (:ACTIVE_NORMAL . 5)
    (:CONFIGURATION . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Base_CapSenseMode)))
    "Constants for message type 'Base_CapSenseMode"
  '((:RESERVED . 0)
    (:INACTIVE . 1)
    (:ACTIVE_AUTO_THRESHOLD . 2)
    (:ACTIVE_NOISE_ATT . 4)
    (:ACTIVE_NORMAL . 5)
    (:CONFIGURATION . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Base_CapSenseMode>) ostream)
  "Serializes a message object of type '<Base_CapSenseMode>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Base_CapSenseMode>) istream)
  "Deserializes a message object of type '<Base_CapSenseMode>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Base_CapSenseMode>)))
  "Returns string type for a message object of type '<Base_CapSenseMode>"
  "kortex_driver/Base_CapSenseMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_CapSenseMode)))
  "Returns string type for a message object of type 'Base_CapSenseMode"
  "kortex_driver/Base_CapSenseMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Base_CapSenseMode>)))
  "Returns md5sum for a message object of type '<Base_CapSenseMode>"
  "ab37ae8a57a132b00bf1d50bed534551")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Base_CapSenseMode)))
  "Returns md5sum for a message object of type 'Base_CapSenseMode"
  "ab37ae8a57a132b00bf1d50bed534551")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Base_CapSenseMode>)))
  "Returns full string definition for message of type '<Base_CapSenseMode>"
  (cl:format cl:nil "~%uint32 RESERVED = 0~%~%uint32 INACTIVE = 1~%~%uint32 ACTIVE_AUTO_THRESHOLD = 2~%~%uint32 ACTIVE_NOISE_ATT = 4~%~%uint32 ACTIVE_NORMAL = 5~%~%uint32 CONFIGURATION = 6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Base_CapSenseMode)))
  "Returns full string definition for message of type 'Base_CapSenseMode"
  (cl:format cl:nil "~%uint32 RESERVED = 0~%~%uint32 INACTIVE = 1~%~%uint32 ACTIVE_AUTO_THRESHOLD = 2~%~%uint32 ACTIVE_NOISE_ATT = 4~%~%uint32 ACTIVE_NORMAL = 5~%~%uint32 CONFIGURATION = 6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Base_CapSenseMode>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Base_CapSenseMode>))
  "Converts a ROS message object to a list"
  (cl:list 'Base_CapSenseMode
))
