; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Unit.msg.html

(cl:defclass <Unit> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Unit (<Unit>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Unit>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Unit)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Unit> is deprecated: use kortex_driver-msg:Unit instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Unit>)))
    "Constants for message type '<Unit>"
  '((:UNSPECIFIED_UNIT . 0)
    (:CELSIUS . 1)
    (:AMPERE . 2)
    (:VOLT . 3)
    (:METER_PER_SECOND . 4)
    (:DEGREE_PER_SECOND . 5)
    (:METER_PER_SECOND_2 . 6)
    (:DEGREE_PER_SECOND_2 . 7)
    (:NEWTON . 8)
    (:NEWTON_METER . 9)
    (:KILOGRAM . 10)
    (:DEGREE . 11)
    (:TICK . 12)
    (:DEGREE_PER_MILLISECOND . 13))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Unit)))
    "Constants for message type 'Unit"
  '((:UNSPECIFIED_UNIT . 0)
    (:CELSIUS . 1)
    (:AMPERE . 2)
    (:VOLT . 3)
    (:METER_PER_SECOND . 4)
    (:DEGREE_PER_SECOND . 5)
    (:METER_PER_SECOND_2 . 6)
    (:DEGREE_PER_SECOND_2 . 7)
    (:NEWTON . 8)
    (:NEWTON_METER . 9)
    (:KILOGRAM . 10)
    (:DEGREE . 11)
    (:TICK . 12)
    (:DEGREE_PER_MILLISECOND . 13))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Unit>) ostream)
  "Serializes a message object of type '<Unit>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Unit>) istream)
  "Deserializes a message object of type '<Unit>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Unit>)))
  "Returns string type for a message object of type '<Unit>"
  "kortex_driver/Unit")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Unit)))
  "Returns string type for a message object of type 'Unit"
  "kortex_driver/Unit")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Unit>)))
  "Returns md5sum for a message object of type '<Unit>"
  "38eeb5ea9959fe1af60e856b88a5f9a7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Unit)))
  "Returns md5sum for a message object of type 'Unit"
  "38eeb5ea9959fe1af60e856b88a5f9a7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Unit>)))
  "Returns full string definition for message of type '<Unit>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_UNIT = 0~%~%uint32 CELSIUS = 1~%~%uint32 AMPERE = 2~%~%uint32 VOLT = 3~%~%uint32 METER_PER_SECOND = 4~%~%uint32 DEGREE_PER_SECOND = 5~%~%uint32 METER_PER_SECOND_2 = 6~%~%uint32 DEGREE_PER_SECOND_2 = 7~%~%uint32 NEWTON = 8~%~%uint32 NEWTON_METER = 9~%~%uint32 KILOGRAM = 10~%~%uint32 DEGREE = 11~%~%uint32 TICK = 12~%~%uint32 DEGREE_PER_MILLISECOND = 13~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Unit)))
  "Returns full string definition for message of type 'Unit"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_UNIT = 0~%~%uint32 CELSIUS = 1~%~%uint32 AMPERE = 2~%~%uint32 VOLT = 3~%~%uint32 METER_PER_SECOND = 4~%~%uint32 DEGREE_PER_SECOND = 5~%~%uint32 METER_PER_SECOND_2 = 6~%~%uint32 DEGREE_PER_SECOND_2 = 7~%~%uint32 NEWTON = 8~%~%uint32 NEWTON_METER = 9~%~%uint32 KILOGRAM = 10~%~%uint32 DEGREE = 11~%~%uint32 TICK = 12~%~%uint32 DEGREE_PER_MILLISECOND = 13~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Unit>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Unit>))
  "Converts a ROS message object to a list"
  (cl:list 'Unit
))
