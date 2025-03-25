; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude UserEvent.msg.html

(cl:defclass <UserEvent> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass UserEvent (<UserEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UserEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UserEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<UserEvent> is deprecated: use kortex_driver-msg:UserEvent instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<UserEvent>)))
    "Constants for message type '<UserEvent>"
  '((:UNSPECIFIED_USER_EVENT . 0)
    (:LOGGED_OUT . 1)
    (:LOGGED_IN . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'UserEvent)))
    "Constants for message type 'UserEvent"
  '((:UNSPECIFIED_USER_EVENT . 0)
    (:LOGGED_OUT . 1)
    (:LOGGED_IN . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UserEvent>) ostream)
  "Serializes a message object of type '<UserEvent>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UserEvent>) istream)
  "Deserializes a message object of type '<UserEvent>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UserEvent>)))
  "Returns string type for a message object of type '<UserEvent>"
  "kortex_driver/UserEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UserEvent)))
  "Returns string type for a message object of type 'UserEvent"
  "kortex_driver/UserEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UserEvent>)))
  "Returns md5sum for a message object of type '<UserEvent>"
  "6b6b55b08fd45b34a43cf1d05e2fdbdc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UserEvent)))
  "Returns md5sum for a message object of type 'UserEvent"
  "6b6b55b08fd45b34a43cf1d05e2fdbdc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UserEvent>)))
  "Returns full string definition for message of type '<UserEvent>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_USER_EVENT = 0~%~%uint32 LOGGED_OUT = 1~%~%uint32 LOGGED_IN = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UserEvent)))
  "Returns full string definition for message of type 'UserEvent"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_USER_EVENT = 0~%~%uint32 LOGGED_OUT = 1~%~%uint32 LOGGED_IN = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UserEvent>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UserEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'UserEvent
))
