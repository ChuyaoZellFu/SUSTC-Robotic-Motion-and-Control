; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude LimitationType.msg.html

(cl:defclass <LimitationType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass LimitationType (<LimitationType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LimitationType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LimitationType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<LimitationType> is deprecated: use kortex_driver-msg:LimitationType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<LimitationType>)))
    "Constants for message type '<LimitationType>"
  '((:UNSPECIFIED_LIMITATION . 0)
    (:FORCE_LIMITATION . 1)
    (:ACCELERATION_LIMITATION . 2)
    (:VELOCITY_LIMITATION . 3)
    (:TORQUE_LIMITATION . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'LimitationType)))
    "Constants for message type 'LimitationType"
  '((:UNSPECIFIED_LIMITATION . 0)
    (:FORCE_LIMITATION . 1)
    (:ACCELERATION_LIMITATION . 2)
    (:VELOCITY_LIMITATION . 3)
    (:TORQUE_LIMITATION . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LimitationType>) ostream)
  "Serializes a message object of type '<LimitationType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LimitationType>) istream)
  "Deserializes a message object of type '<LimitationType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LimitationType>)))
  "Returns string type for a message object of type '<LimitationType>"
  "kortex_driver/LimitationType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LimitationType)))
  "Returns string type for a message object of type 'LimitationType"
  "kortex_driver/LimitationType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LimitationType>)))
  "Returns md5sum for a message object of type '<LimitationType>"
  "86bbe126d2c218724be7709cc366b9d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LimitationType)))
  "Returns md5sum for a message object of type 'LimitationType"
  "86bbe126d2c218724be7709cc366b9d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LimitationType>)))
  "Returns full string definition for message of type '<LimitationType>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_LIMITATION = 0~%~%uint32 FORCE_LIMITATION = 1~%~%uint32 ACCELERATION_LIMITATION = 2~%~%uint32 VELOCITY_LIMITATION = 3~%~%uint32 TORQUE_LIMITATION = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LimitationType)))
  "Returns full string definition for message of type 'LimitationType"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_LIMITATION = 0~%~%uint32 FORCE_LIMITATION = 1~%~%uint32 ACCELERATION_LIMITATION = 2~%~%uint32 VELOCITY_LIMITATION = 3~%~%uint32 TORQUE_LIMITATION = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LimitationType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LimitationType>))
  "Converts a ROS message object to a list"
  (cl:list 'LimitationType
))
