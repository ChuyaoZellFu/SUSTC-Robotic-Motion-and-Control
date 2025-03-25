; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ServoingMode.msg.html

(cl:defclass <ServoingMode> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ServoingMode (<ServoingMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ServoingMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ServoingMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ServoingMode> is deprecated: use kortex_driver-msg:ServoingMode instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ServoingMode>)))
    "Constants for message type '<ServoingMode>"
  '((:UNSPECIFIED_SERVOING_MODE . 0)
    (:SINGLE_LEVEL_SERVOING . 2)
    (:LOW_LEVEL_SERVOING . 3)
    (:BYPASS_SERVOING . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ServoingMode)))
    "Constants for message type 'ServoingMode"
  '((:UNSPECIFIED_SERVOING_MODE . 0)
    (:SINGLE_LEVEL_SERVOING . 2)
    (:LOW_LEVEL_SERVOING . 3)
    (:BYPASS_SERVOING . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ServoingMode>) ostream)
  "Serializes a message object of type '<ServoingMode>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ServoingMode>) istream)
  "Deserializes a message object of type '<ServoingMode>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ServoingMode>)))
  "Returns string type for a message object of type '<ServoingMode>"
  "kortex_driver/ServoingMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ServoingMode)))
  "Returns string type for a message object of type 'ServoingMode"
  "kortex_driver/ServoingMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ServoingMode>)))
  "Returns md5sum for a message object of type '<ServoingMode>"
  "3ce819b6b24c3de6343a9b4a6de953eb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ServoingMode)))
  "Returns md5sum for a message object of type 'ServoingMode"
  "3ce819b6b24c3de6343a9b4a6de953eb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ServoingMode>)))
  "Returns full string definition for message of type '<ServoingMode>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_SERVOING_MODE = 0~%~%uint32 SINGLE_LEVEL_SERVOING = 2~%~%uint32 LOW_LEVEL_SERVOING = 3~%~%uint32 BYPASS_SERVOING = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ServoingMode)))
  "Returns full string definition for message of type 'ServoingMode"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_SERVOING_MODE = 0~%~%uint32 SINGLE_LEVEL_SERVOING = 2~%~%uint32 LOW_LEVEL_SERVOING = 3~%~%uint32 BYPASS_SERVOING = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ServoingMode>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ServoingMode>))
  "Converts a ROS message object to a list"
  (cl:list 'ServoingMode
))
