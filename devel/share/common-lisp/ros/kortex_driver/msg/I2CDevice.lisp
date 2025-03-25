; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude I2CDevice.msg.html

(cl:defclass <I2CDevice> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass I2CDevice (<I2CDevice>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CDevice>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CDevice)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<I2CDevice> is deprecated: use kortex_driver-msg:I2CDevice instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<I2CDevice>)))
    "Constants for message type '<I2CDevice>"
  '((:I2C_DEVICE_UNSPECIFIED . 0)
    (:I2C_DEVICE_EXPANSION . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'I2CDevice)))
    "Constants for message type 'I2CDevice"
  '((:I2C_DEVICE_UNSPECIFIED . 0)
    (:I2C_DEVICE_EXPANSION . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CDevice>) ostream)
  "Serializes a message object of type '<I2CDevice>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CDevice>) istream)
  "Deserializes a message object of type '<I2CDevice>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CDevice>)))
  "Returns string type for a message object of type '<I2CDevice>"
  "kortex_driver/I2CDevice")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CDevice)))
  "Returns string type for a message object of type 'I2CDevice"
  "kortex_driver/I2CDevice")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CDevice>)))
  "Returns md5sum for a message object of type '<I2CDevice>"
  "f989991b5a55a050960f41f9ed3a70ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CDevice)))
  "Returns md5sum for a message object of type 'I2CDevice"
  "f989991b5a55a050960f41f9ed3a70ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CDevice>)))
  "Returns full string definition for message of type '<I2CDevice>"
  (cl:format cl:nil "~%uint32 I2C_DEVICE_UNSPECIFIED = 0~%~%uint32 I2C_DEVICE_EXPANSION = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CDevice)))
  "Returns full string definition for message of type 'I2CDevice"
  (cl:format cl:nil "~%uint32 I2C_DEVICE_UNSPECIFIED = 0~%~%uint32 I2C_DEVICE_EXPANSION = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CDevice>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CDevice>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CDevice
))
