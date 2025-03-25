; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CommandFlags.msg.html

(cl:defclass <CommandFlags> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CommandFlags (<CommandFlags>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommandFlags>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommandFlags)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CommandFlags> is deprecated: use kortex_driver-msg:CommandFlags instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CommandFlags>)))
    "Constants for message type '<CommandFlags>"
  '((:NO_COMMAND . 0)
    (:SERVO_ENABLE . 1)
    (:BRAKE_DISABLE . 2)
    (:CLEAR_MAJOR_FAULT . 4)
    (:CLEAR_MINOR_FAULT . 8)
    (:PROTECTIVE_STOP . 16)
    (:FORCE_BRAKE_RELEASE . 32)
    (:IGNORE . 64)
    (:LOW_GAINS . 128)
    (:LED_0 . 256)
    (:LED_1 . 512))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CommandFlags)))
    "Constants for message type 'CommandFlags"
  '((:NO_COMMAND . 0)
    (:SERVO_ENABLE . 1)
    (:BRAKE_DISABLE . 2)
    (:CLEAR_MAJOR_FAULT . 4)
    (:CLEAR_MINOR_FAULT . 8)
    (:PROTECTIVE_STOP . 16)
    (:FORCE_BRAKE_RELEASE . 32)
    (:IGNORE . 64)
    (:LOW_GAINS . 128)
    (:LED_0 . 256)
    (:LED_1 . 512))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommandFlags>) ostream)
  "Serializes a message object of type '<CommandFlags>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommandFlags>) istream)
  "Deserializes a message object of type '<CommandFlags>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommandFlags>)))
  "Returns string type for a message object of type '<CommandFlags>"
  "kortex_driver/CommandFlags")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandFlags)))
  "Returns string type for a message object of type 'CommandFlags"
  "kortex_driver/CommandFlags")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommandFlags>)))
  "Returns md5sum for a message object of type '<CommandFlags>"
  "e51ff7d8ad00c576e3aaa1ef0c879bf9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommandFlags)))
  "Returns md5sum for a message object of type 'CommandFlags"
  "e51ff7d8ad00c576e3aaa1ef0c879bf9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommandFlags>)))
  "Returns full string definition for message of type '<CommandFlags>"
  (cl:format cl:nil "~%uint32 NO_COMMAND = 0~%~%uint32 SERVO_ENABLE = 1~%~%uint32 BRAKE_DISABLE = 2~%~%uint32 CLEAR_MAJOR_FAULT = 4~%~%uint32 CLEAR_MINOR_FAULT = 8~%~%uint32 PROTECTIVE_STOP = 16~%~%uint32 FORCE_BRAKE_RELEASE = 32~%~%uint32 IGNORE = 64~%~%uint32 LOW_GAINS = 128~%~%uint32 LED_0 = 256~%~%uint32 LED_1 = 512~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommandFlags)))
  "Returns full string definition for message of type 'CommandFlags"
  (cl:format cl:nil "~%uint32 NO_COMMAND = 0~%~%uint32 SERVO_ENABLE = 1~%~%uint32 BRAKE_DISABLE = 2~%~%uint32 CLEAR_MAJOR_FAULT = 4~%~%uint32 CLEAR_MINOR_FAULT = 8~%~%uint32 PROTECTIVE_STOP = 16~%~%uint32 FORCE_BRAKE_RELEASE = 32~%~%uint32 IGNORE = 64~%~%uint32 LOW_GAINS = 128~%~%uint32 LED_0 = 256~%~%uint32 LED_1 = 512~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommandFlags>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommandFlags>))
  "Converts a ROS message object to a list"
  (cl:list 'CommandFlags
))
