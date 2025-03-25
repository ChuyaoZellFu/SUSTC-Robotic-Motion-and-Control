; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude WifiSecurityType.msg.html

(cl:defclass <WifiSecurityType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass WifiSecurityType (<WifiSecurityType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WifiSecurityType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WifiSecurityType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<WifiSecurityType> is deprecated: use kortex_driver-msg:WifiSecurityType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<WifiSecurityType>)))
    "Constants for message type '<WifiSecurityType>"
  '((:UNSPECIFIED_AUTHENTICATION . 0)
    (:WEP . 1)
    (:WPA2_PERSONAL . 2)
    (:WPA_PERSONAL . 4)
    (:NO_AUTHENTICATION . 8))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'WifiSecurityType)))
    "Constants for message type 'WifiSecurityType"
  '((:UNSPECIFIED_AUTHENTICATION . 0)
    (:WEP . 1)
    (:WPA2_PERSONAL . 2)
    (:WPA_PERSONAL . 4)
    (:NO_AUTHENTICATION . 8))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WifiSecurityType>) ostream)
  "Serializes a message object of type '<WifiSecurityType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WifiSecurityType>) istream)
  "Deserializes a message object of type '<WifiSecurityType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WifiSecurityType>)))
  "Returns string type for a message object of type '<WifiSecurityType>"
  "kortex_driver/WifiSecurityType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WifiSecurityType)))
  "Returns string type for a message object of type 'WifiSecurityType"
  "kortex_driver/WifiSecurityType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WifiSecurityType>)))
  "Returns md5sum for a message object of type '<WifiSecurityType>"
  "758c13b30796cf34636bc4ccee207084")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WifiSecurityType)))
  "Returns md5sum for a message object of type 'WifiSecurityType"
  "758c13b30796cf34636bc4ccee207084")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WifiSecurityType>)))
  "Returns full string definition for message of type '<WifiSecurityType>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_AUTHENTICATION = 0~%~%uint32 WEP = 1~%~%uint32 WPA2_PERSONAL = 2~%~%uint32 WPA_PERSONAL = 4~%~%uint32 NO_AUTHENTICATION = 8~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WifiSecurityType)))
  "Returns full string definition for message of type 'WifiSecurityType"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_AUTHENTICATION = 0~%~%uint32 WEP = 1~%~%uint32 WPA2_PERSONAL = 2~%~%uint32 WPA_PERSONAL = 4~%~%uint32 NO_AUTHENTICATION = 8~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WifiSecurityType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WifiSecurityType>))
  "Converts a ROS message object to a list"
  (cl:list 'WifiSecurityType
))
