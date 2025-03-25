; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude WifiEncryptionType.msg.html

(cl:defclass <WifiEncryptionType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass WifiEncryptionType (<WifiEncryptionType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WifiEncryptionType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WifiEncryptionType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<WifiEncryptionType> is deprecated: use kortex_driver-msg:WifiEncryptionType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<WifiEncryptionType>)))
    "Constants for message type '<WifiEncryptionType>"
  '((:UNSPECIFIED_ENCRYPTION . 0)
    (:AES_ENCRYPTION . 1)
    (:TKIP_ENCRYPTION . 2)
    (:WEP_ENCRYPTION . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'WifiEncryptionType)))
    "Constants for message type 'WifiEncryptionType"
  '((:UNSPECIFIED_ENCRYPTION . 0)
    (:AES_ENCRYPTION . 1)
    (:TKIP_ENCRYPTION . 2)
    (:WEP_ENCRYPTION . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WifiEncryptionType>) ostream)
  "Serializes a message object of type '<WifiEncryptionType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WifiEncryptionType>) istream)
  "Deserializes a message object of type '<WifiEncryptionType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WifiEncryptionType>)))
  "Returns string type for a message object of type '<WifiEncryptionType>"
  "kortex_driver/WifiEncryptionType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WifiEncryptionType)))
  "Returns string type for a message object of type 'WifiEncryptionType"
  "kortex_driver/WifiEncryptionType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WifiEncryptionType>)))
  "Returns md5sum for a message object of type '<WifiEncryptionType>"
  "5312088e9fe010ba1511377fa25b980d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WifiEncryptionType)))
  "Returns md5sum for a message object of type 'WifiEncryptionType"
  "5312088e9fe010ba1511377fa25b980d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WifiEncryptionType>)))
  "Returns full string definition for message of type '<WifiEncryptionType>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_ENCRYPTION = 0~%~%uint32 AES_ENCRYPTION = 1~%~%uint32 TKIP_ENCRYPTION = 2~%~%uint32 WEP_ENCRYPTION = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WifiEncryptionType)))
  "Returns full string definition for message of type 'WifiEncryptionType"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_ENCRYPTION = 0~%~%uint32 AES_ENCRYPTION = 1~%~%uint32 TKIP_ENCRYPTION = 2~%~%uint32 WEP_ENCRYPTION = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WifiEncryptionType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WifiEncryptionType>))
  "Converts a ROS message object to a list"
  (cl:list 'WifiEncryptionType
))
