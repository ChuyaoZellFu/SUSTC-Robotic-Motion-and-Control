; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude I2CDeviceAddressing.msg.html

(cl:defclass <I2CDeviceAddressing> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass I2CDeviceAddressing (<I2CDeviceAddressing>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CDeviceAddressing>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CDeviceAddressing)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<I2CDeviceAddressing> is deprecated: use kortex_driver-msg:I2CDeviceAddressing instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<I2CDeviceAddressing>)))
    "Constants for message type '<I2CDeviceAddressing>"
  '((:I2C_DEVICE_ADDRESSING_UNSPECIFIED . 0)
    (:I2C_DEVICE_ADDRESSING_7_BITS . 1)
    (:I2C_DEVICE_ADDRESSING_10_BITS . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'I2CDeviceAddressing)))
    "Constants for message type 'I2CDeviceAddressing"
  '((:I2C_DEVICE_ADDRESSING_UNSPECIFIED . 0)
    (:I2C_DEVICE_ADDRESSING_7_BITS . 1)
    (:I2C_DEVICE_ADDRESSING_10_BITS . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CDeviceAddressing>) ostream)
  "Serializes a message object of type '<I2CDeviceAddressing>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CDeviceAddressing>) istream)
  "Deserializes a message object of type '<I2CDeviceAddressing>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CDeviceAddressing>)))
  "Returns string type for a message object of type '<I2CDeviceAddressing>"
  "kortex_driver/I2CDeviceAddressing")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CDeviceAddressing)))
  "Returns string type for a message object of type 'I2CDeviceAddressing"
  "kortex_driver/I2CDeviceAddressing")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CDeviceAddressing>)))
  "Returns md5sum for a message object of type '<I2CDeviceAddressing>"
  "a6df7de78b1ed95513b026e1947c58a0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CDeviceAddressing)))
  "Returns md5sum for a message object of type 'I2CDeviceAddressing"
  "a6df7de78b1ed95513b026e1947c58a0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CDeviceAddressing>)))
  "Returns full string definition for message of type '<I2CDeviceAddressing>"
  (cl:format cl:nil "~%uint32 I2C_DEVICE_ADDRESSING_UNSPECIFIED = 0~%~%uint32 I2C_DEVICE_ADDRESSING_7_BITS = 1~%~%uint32 I2C_DEVICE_ADDRESSING_10_BITS = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CDeviceAddressing)))
  "Returns full string definition for message of type 'I2CDeviceAddressing"
  (cl:format cl:nil "~%uint32 I2C_DEVICE_ADDRESSING_UNSPECIFIED = 0~%~%uint32 I2C_DEVICE_ADDRESSING_7_BITS = 1~%~%uint32 I2C_DEVICE_ADDRESSING_10_BITS = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CDeviceAddressing>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CDeviceAddressing>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CDeviceAddressing
))
