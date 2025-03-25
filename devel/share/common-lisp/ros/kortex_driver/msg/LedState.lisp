; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude LedState.msg.html

(cl:defclass <LedState> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass LedState (<LedState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LedState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LedState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<LedState> is deprecated: use kortex_driver-msg:LedState instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<LedState>)))
    "Constants for message type '<LedState>"
  '((:UNSPECIFIED_LED_STATE . 0)
    (:LED_OFF . 1)
    (:LED_PULSE . 2)
    (:LED_ON . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'LedState)))
    "Constants for message type 'LedState"
  '((:UNSPECIFIED_LED_STATE . 0)
    (:LED_OFF . 1)
    (:LED_PULSE . 2)
    (:LED_ON . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LedState>) ostream)
  "Serializes a message object of type '<LedState>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LedState>) istream)
  "Deserializes a message object of type '<LedState>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LedState>)))
  "Returns string type for a message object of type '<LedState>"
  "kortex_driver/LedState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LedState)))
  "Returns string type for a message object of type 'LedState"
  "kortex_driver/LedState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LedState>)))
  "Returns md5sum for a message object of type '<LedState>"
  "d10da8147549713ab2f5b5d7555fb6b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LedState)))
  "Returns md5sum for a message object of type 'LedState"
  "d10da8147549713ab2f5b5d7555fb6b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LedState>)))
  "Returns full string definition for message of type '<LedState>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_LED_STATE = 0~%~%uint32 LED_OFF = 1~%~%uint32 LED_PULSE = 2~%~%uint32 LED_ON = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LedState)))
  "Returns full string definition for message of type 'LedState"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_LED_STATE = 0~%~%uint32 LED_OFF = 1~%~%uint32 LED_PULSE = 2~%~%uint32 LED_ON = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LedState>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LedState>))
  "Converts a ROS message object to a list"
  (cl:list 'LedState
))
