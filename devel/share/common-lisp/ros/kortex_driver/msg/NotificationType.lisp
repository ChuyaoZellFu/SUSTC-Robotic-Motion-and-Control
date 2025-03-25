; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude NotificationType.msg.html

(cl:defclass <NotificationType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass NotificationType (<NotificationType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NotificationType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NotificationType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<NotificationType> is deprecated: use kortex_driver-msg:NotificationType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<NotificationType>)))
    "Constants for message type '<NotificationType>"
  '((:NOTIFICATION_TYPE_UNSPECIFIED . 0)
    (:NOTIFICATION_TYPE_THRESHOLD . 1)
    (:NOTIFICATION_TYPE_FIX_RATE . 2)
    (:NOTIFICATION_TYPE_EVENT . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'NotificationType)))
    "Constants for message type 'NotificationType"
  '((:NOTIFICATION_TYPE_UNSPECIFIED . 0)
    (:NOTIFICATION_TYPE_THRESHOLD . 1)
    (:NOTIFICATION_TYPE_FIX_RATE . 2)
    (:NOTIFICATION_TYPE_EVENT . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NotificationType>) ostream)
  "Serializes a message object of type '<NotificationType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NotificationType>) istream)
  "Deserializes a message object of type '<NotificationType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NotificationType>)))
  "Returns string type for a message object of type '<NotificationType>"
  "kortex_driver/NotificationType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NotificationType)))
  "Returns string type for a message object of type 'NotificationType"
  "kortex_driver/NotificationType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NotificationType>)))
  "Returns md5sum for a message object of type '<NotificationType>"
  "9d8153f0fe98641698596673829b2649")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NotificationType)))
  "Returns md5sum for a message object of type 'NotificationType"
  "9d8153f0fe98641698596673829b2649")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NotificationType>)))
  "Returns full string definition for message of type '<NotificationType>"
  (cl:format cl:nil "~%uint32 NOTIFICATION_TYPE_UNSPECIFIED = 0~%~%uint32 NOTIFICATION_TYPE_THRESHOLD = 1~%~%uint32 NOTIFICATION_TYPE_FIX_RATE = 2~%~%uint32 NOTIFICATION_TYPE_EVENT = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NotificationType)))
  "Returns full string definition for message of type 'NotificationType"
  (cl:format cl:nil "~%uint32 NOTIFICATION_TYPE_UNSPECIFIED = 0~%~%uint32 NOTIFICATION_TYPE_THRESHOLD = 1~%~%uint32 NOTIFICATION_TYPE_FIX_RATE = 2~%~%uint32 NOTIFICATION_TYPE_EVENT = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NotificationType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NotificationType>))
  "Converts a ROS message object to a list"
  (cl:list 'NotificationType
))
