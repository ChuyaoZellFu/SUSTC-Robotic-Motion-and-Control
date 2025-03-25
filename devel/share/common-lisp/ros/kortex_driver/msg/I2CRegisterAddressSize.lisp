; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude I2CRegisterAddressSize.msg.html

(cl:defclass <I2CRegisterAddressSize> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass I2CRegisterAddressSize (<I2CRegisterAddressSize>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CRegisterAddressSize>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CRegisterAddressSize)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<I2CRegisterAddressSize> is deprecated: use kortex_driver-msg:I2CRegisterAddressSize instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<I2CRegisterAddressSize>)))
    "Constants for message type '<I2CRegisterAddressSize>"
  '((:I2C_REGISTER_ADDRESS_SIZE_UNSPECIFIED . 0)
    (:I2C_REGISTER_ADDRESS_SIZE_8_BITS . 1)
    (:I2C_REGISTER_ADDRESS_SIZE_16_BITS . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'I2CRegisterAddressSize)))
    "Constants for message type 'I2CRegisterAddressSize"
  '((:I2C_REGISTER_ADDRESS_SIZE_UNSPECIFIED . 0)
    (:I2C_REGISTER_ADDRESS_SIZE_8_BITS . 1)
    (:I2C_REGISTER_ADDRESS_SIZE_16_BITS . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CRegisterAddressSize>) ostream)
  "Serializes a message object of type '<I2CRegisterAddressSize>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CRegisterAddressSize>) istream)
  "Deserializes a message object of type '<I2CRegisterAddressSize>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CRegisterAddressSize>)))
  "Returns string type for a message object of type '<I2CRegisterAddressSize>"
  "kortex_driver/I2CRegisterAddressSize")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CRegisterAddressSize)))
  "Returns string type for a message object of type 'I2CRegisterAddressSize"
  "kortex_driver/I2CRegisterAddressSize")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CRegisterAddressSize>)))
  "Returns md5sum for a message object of type '<I2CRegisterAddressSize>"
  "517dd96a7b250066663891b1fad6bd8a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CRegisterAddressSize)))
  "Returns md5sum for a message object of type 'I2CRegisterAddressSize"
  "517dd96a7b250066663891b1fad6bd8a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CRegisterAddressSize>)))
  "Returns full string definition for message of type '<I2CRegisterAddressSize>"
  (cl:format cl:nil "~%uint32 I2C_REGISTER_ADDRESS_SIZE_UNSPECIFIED = 0~%~%uint32 I2C_REGISTER_ADDRESS_SIZE_8_BITS = 1~%~%uint32 I2C_REGISTER_ADDRESS_SIZE_16_BITS = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CRegisterAddressSize)))
  "Returns full string definition for message of type 'I2CRegisterAddressSize"
  (cl:format cl:nil "~%uint32 I2C_REGISTER_ADDRESS_SIZE_UNSPECIFIED = 0~%~%uint32 I2C_REGISTER_ADDRESS_SIZE_8_BITS = 1~%~%uint32 I2C_REGISTER_ADDRESS_SIZE_16_BITS = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CRegisterAddressSize>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CRegisterAddressSize>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CRegisterAddressSize
))
