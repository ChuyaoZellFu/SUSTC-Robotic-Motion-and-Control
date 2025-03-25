; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ErrorCodes.msg.html

(cl:defclass <ErrorCodes> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ErrorCodes (<ErrorCodes>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ErrorCodes>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ErrorCodes)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ErrorCodes> is deprecated: use kortex_driver-msg:ErrorCodes instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ErrorCodes>)))
    "Constants for message type '<ErrorCodes>"
  '((:ERROR_NONE . 0)
    (:ERROR_PROTOCOL_SERVER . 1)
    (:ERROR_PROTOCOL_CLIENT . 2)
    (:ERROR_DEVICE . 3)
    (:ERROR_INTERNAL . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ErrorCodes)))
    "Constants for message type 'ErrorCodes"
  '((:ERROR_NONE . 0)
    (:ERROR_PROTOCOL_SERVER . 1)
    (:ERROR_PROTOCOL_CLIENT . 2)
    (:ERROR_DEVICE . 3)
    (:ERROR_INTERNAL . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ErrorCodes>) ostream)
  "Serializes a message object of type '<ErrorCodes>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ErrorCodes>) istream)
  "Deserializes a message object of type '<ErrorCodes>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ErrorCodes>)))
  "Returns string type for a message object of type '<ErrorCodes>"
  "kortex_driver/ErrorCodes")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ErrorCodes)))
  "Returns string type for a message object of type 'ErrorCodes"
  "kortex_driver/ErrorCodes")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ErrorCodes>)))
  "Returns md5sum for a message object of type '<ErrorCodes>"
  "cb78b6e4994aaf216b83b68ecaa14cbd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ErrorCodes)))
  "Returns md5sum for a message object of type 'ErrorCodes"
  "cb78b6e4994aaf216b83b68ecaa14cbd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ErrorCodes>)))
  "Returns full string definition for message of type '<ErrorCodes>"
  (cl:format cl:nil "~%uint32 ERROR_NONE = 0~%~%uint32 ERROR_PROTOCOL_SERVER = 1~%~%uint32 ERROR_PROTOCOL_CLIENT = 2~%~%uint32 ERROR_DEVICE = 3~%~%uint32 ERROR_INTERNAL = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ErrorCodes)))
  "Returns full string definition for message of type 'ErrorCodes"
  (cl:format cl:nil "~%uint32 ERROR_NONE = 0~%~%uint32 ERROR_PROTOCOL_SERVER = 1~%~%uint32 ERROR_PROTOCOL_CLIENT = 2~%~%uint32 ERROR_DEVICE = 3~%~%uint32 ERROR_INTERNAL = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ErrorCodes>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ErrorCodes>))
  "Converts a ROS message object to a list"
  (cl:list 'ErrorCodes
))
