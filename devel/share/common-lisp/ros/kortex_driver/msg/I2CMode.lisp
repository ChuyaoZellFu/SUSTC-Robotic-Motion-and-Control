; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude I2CMode.msg.html

(cl:defclass <I2CMode> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass I2CMode (<I2CMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<I2CMode> is deprecated: use kortex_driver-msg:I2CMode instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<I2CMode>)))
    "Constants for message type '<I2CMode>"
  '((:I2C_MODE_UNSPECIFIED . 0)
    (:I2C_MODE_STANDARD . 1)
    (:I2C_MODE_FAST . 2)
    (:I2C_MODE_FAST_PLUS . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'I2CMode)))
    "Constants for message type 'I2CMode"
  '((:I2C_MODE_UNSPECIFIED . 0)
    (:I2C_MODE_STANDARD . 1)
    (:I2C_MODE_FAST . 2)
    (:I2C_MODE_FAST_PLUS . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CMode>) ostream)
  "Serializes a message object of type '<I2CMode>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CMode>) istream)
  "Deserializes a message object of type '<I2CMode>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CMode>)))
  "Returns string type for a message object of type '<I2CMode>"
  "kortex_driver/I2CMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CMode)))
  "Returns string type for a message object of type 'I2CMode"
  "kortex_driver/I2CMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CMode>)))
  "Returns md5sum for a message object of type '<I2CMode>"
  "56d9e828030957dbebbc8d714433e077")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CMode)))
  "Returns md5sum for a message object of type 'I2CMode"
  "56d9e828030957dbebbc8d714433e077")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CMode>)))
  "Returns full string definition for message of type '<I2CMode>"
  (cl:format cl:nil "~%uint32 I2C_MODE_UNSPECIFIED = 0~%~%uint32 I2C_MODE_STANDARD = 1~%~%uint32 I2C_MODE_FAST = 2~%~%uint32 I2C_MODE_FAST_PLUS = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CMode)))
  "Returns full string definition for message of type 'I2CMode"
  (cl:format cl:nil "~%uint32 I2C_MODE_UNSPECIFIED = 0~%~%uint32 I2C_MODE_STANDARD = 1~%~%uint32 I2C_MODE_FAST = 2~%~%uint32 I2C_MODE_FAST_PLUS = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CMode>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CMode>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CMode
))
