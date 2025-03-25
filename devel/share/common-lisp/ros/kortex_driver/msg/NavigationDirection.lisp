; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude NavigationDirection.msg.html

(cl:defclass <NavigationDirection> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass NavigationDirection (<NavigationDirection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NavigationDirection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NavigationDirection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<NavigationDirection> is deprecated: use kortex_driver-msg:NavigationDirection instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<NavigationDirection>)))
    "Constants for message type '<NavigationDirection>"
  '((:UNSPECIFIED_NAVIGATION_DIRECTION . 0)
    (:NEXT . 1)
    (:UP . 2)
    (:DOWN . 3)
    (:PREVIOUS . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'NavigationDirection)))
    "Constants for message type 'NavigationDirection"
  '((:UNSPECIFIED_NAVIGATION_DIRECTION . 0)
    (:NEXT . 1)
    (:UP . 2)
    (:DOWN . 3)
    (:PREVIOUS . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NavigationDirection>) ostream)
  "Serializes a message object of type '<NavigationDirection>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NavigationDirection>) istream)
  "Deserializes a message object of type '<NavigationDirection>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NavigationDirection>)))
  "Returns string type for a message object of type '<NavigationDirection>"
  "kortex_driver/NavigationDirection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NavigationDirection)))
  "Returns string type for a message object of type 'NavigationDirection"
  "kortex_driver/NavigationDirection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NavigationDirection>)))
  "Returns md5sum for a message object of type '<NavigationDirection>"
  "db0163da7423639bdc46c2cceb491c80")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NavigationDirection)))
  "Returns md5sum for a message object of type 'NavigationDirection"
  "db0163da7423639bdc46c2cceb491c80")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NavigationDirection>)))
  "Returns full string definition for message of type '<NavigationDirection>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_NAVIGATION_DIRECTION = 0~%~%uint32 NEXT = 1~%~%uint32 UP = 2~%~%uint32 DOWN = 3~%~%uint32 PREVIOUS = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NavigationDirection)))
  "Returns full string definition for message of type 'NavigationDirection"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_NAVIGATION_DIRECTION = 0~%~%uint32 NEXT = 1~%~%uint32 UP = 2~%~%uint32 DOWN = 3~%~%uint32 PREVIOUS = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NavigationDirection>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NavigationDirection>))
  "Converts a ROS message object to a list"
  (cl:list 'NavigationDirection
))
