; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ProtectionZoneEvent.msg.html

(cl:defclass <ProtectionZoneEvent> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ProtectionZoneEvent (<ProtectionZoneEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProtectionZoneEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProtectionZoneEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ProtectionZoneEvent> is deprecated: use kortex_driver-msg:ProtectionZoneEvent instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ProtectionZoneEvent>)))
    "Constants for message type '<ProtectionZoneEvent>"
  '((:UNSPECIFIED_PROTECTION_ZONE_EVENT . 0)
    (:REACHED . 1)
    (:ENTERED . 2)
    (:EXITED . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ProtectionZoneEvent)))
    "Constants for message type 'ProtectionZoneEvent"
  '((:UNSPECIFIED_PROTECTION_ZONE_EVENT . 0)
    (:REACHED . 1)
    (:ENTERED . 2)
    (:EXITED . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProtectionZoneEvent>) ostream)
  "Serializes a message object of type '<ProtectionZoneEvent>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProtectionZoneEvent>) istream)
  "Deserializes a message object of type '<ProtectionZoneEvent>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProtectionZoneEvent>)))
  "Returns string type for a message object of type '<ProtectionZoneEvent>"
  "kortex_driver/ProtectionZoneEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProtectionZoneEvent)))
  "Returns string type for a message object of type 'ProtectionZoneEvent"
  "kortex_driver/ProtectionZoneEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProtectionZoneEvent>)))
  "Returns md5sum for a message object of type '<ProtectionZoneEvent>"
  "9e1a0e125d0c3437f9fc79af445cf181")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProtectionZoneEvent)))
  "Returns md5sum for a message object of type 'ProtectionZoneEvent"
  "9e1a0e125d0c3437f9fc79af445cf181")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProtectionZoneEvent>)))
  "Returns full string definition for message of type '<ProtectionZoneEvent>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_PROTECTION_ZONE_EVENT = 0~%~%uint32 REACHED = 1~%~%uint32 ENTERED = 2~%~%uint32 EXITED = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProtectionZoneEvent)))
  "Returns full string definition for message of type 'ProtectionZoneEvent"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_PROTECTION_ZONE_EVENT = 0~%~%uint32 REACHED = 1~%~%uint32 ENTERED = 2~%~%uint32 EXITED = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProtectionZoneEvent>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProtectionZoneEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'ProtectionZoneEvent
))
