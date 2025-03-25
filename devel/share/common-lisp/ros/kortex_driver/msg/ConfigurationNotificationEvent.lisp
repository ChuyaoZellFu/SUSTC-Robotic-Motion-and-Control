; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ConfigurationNotificationEvent.msg.html

(cl:defclass <ConfigurationNotificationEvent> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ConfigurationNotificationEvent (<ConfigurationNotificationEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigurationNotificationEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigurationNotificationEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ConfigurationNotificationEvent> is deprecated: use kortex_driver-msg:ConfigurationNotificationEvent instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ConfigurationNotificationEvent>)))
    "Constants for message type '<ConfigurationNotificationEvent>"
  '((:UNSPECIFIED_CONFIGURATION_EVENT . 0)
    (:CONFIGURATION_UPDATED . 1)
    (:CONFIGURATION_DELETED . 2)
    (:CONFIGURATION_DELETED_ALL . 3)
    (:CONFIGURATION_CREATED . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ConfigurationNotificationEvent)))
    "Constants for message type 'ConfigurationNotificationEvent"
  '((:UNSPECIFIED_CONFIGURATION_EVENT . 0)
    (:CONFIGURATION_UPDATED . 1)
    (:CONFIGURATION_DELETED . 2)
    (:CONFIGURATION_DELETED_ALL . 3)
    (:CONFIGURATION_CREATED . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigurationNotificationEvent>) ostream)
  "Serializes a message object of type '<ConfigurationNotificationEvent>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigurationNotificationEvent>) istream)
  "Deserializes a message object of type '<ConfigurationNotificationEvent>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigurationNotificationEvent>)))
  "Returns string type for a message object of type '<ConfigurationNotificationEvent>"
  "kortex_driver/ConfigurationNotificationEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigurationNotificationEvent)))
  "Returns string type for a message object of type 'ConfigurationNotificationEvent"
  "kortex_driver/ConfigurationNotificationEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigurationNotificationEvent>)))
  "Returns md5sum for a message object of type '<ConfigurationNotificationEvent>"
  "c8a6a5a55ab6b7b3399179fb55277b67")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigurationNotificationEvent)))
  "Returns md5sum for a message object of type 'ConfigurationNotificationEvent"
  "c8a6a5a55ab6b7b3399179fb55277b67")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigurationNotificationEvent>)))
  "Returns full string definition for message of type '<ConfigurationNotificationEvent>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CONFIGURATION_EVENT = 0~%~%uint32 CONFIGURATION_UPDATED = 1~%~%uint32 CONFIGURATION_DELETED = 2~%~%uint32 CONFIGURATION_DELETED_ALL = 3~%~%uint32 CONFIGURATION_CREATED = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigurationNotificationEvent)))
  "Returns full string definition for message of type 'ConfigurationNotificationEvent"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CONFIGURATION_EVENT = 0~%~%uint32 CONFIGURATION_UPDATED = 1~%~%uint32 CONFIGURATION_DELETED = 2~%~%uint32 CONFIGURATION_DELETED_ALL = 3~%~%uint32 CONFIGURATION_CREATED = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigurationNotificationEvent>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigurationNotificationEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigurationNotificationEvent
))
