; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude FactoryEvent.msg.html

(cl:defclass <FactoryEvent> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass FactoryEvent (<FactoryEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FactoryEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FactoryEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<FactoryEvent> is deprecated: use kortex_driver-msg:FactoryEvent instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<FactoryEvent>)))
    "Constants for message type '<FactoryEvent>"
  '((:UNSPECIFIED_FACTORY_EVENT . 0)
    (:FACTORY_DEFAULT_RESTORED . 1)
    (:NETWORK_FACTORY_DEFAULT_RESTORED . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'FactoryEvent)))
    "Constants for message type 'FactoryEvent"
  '((:UNSPECIFIED_FACTORY_EVENT . 0)
    (:FACTORY_DEFAULT_RESTORED . 1)
    (:NETWORK_FACTORY_DEFAULT_RESTORED . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FactoryEvent>) ostream)
  "Serializes a message object of type '<FactoryEvent>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FactoryEvent>) istream)
  "Deserializes a message object of type '<FactoryEvent>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FactoryEvent>)))
  "Returns string type for a message object of type '<FactoryEvent>"
  "kortex_driver/FactoryEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FactoryEvent)))
  "Returns string type for a message object of type 'FactoryEvent"
  "kortex_driver/FactoryEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FactoryEvent>)))
  "Returns md5sum for a message object of type '<FactoryEvent>"
  "f21e922380f9063597fbdefcc7ac55fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FactoryEvent)))
  "Returns md5sum for a message object of type 'FactoryEvent"
  "f21e922380f9063597fbdefcc7ac55fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FactoryEvent>)))
  "Returns full string definition for message of type '<FactoryEvent>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_FACTORY_EVENT = 0~%~%uint32 FACTORY_DEFAULT_RESTORED = 1~%~%uint32 NETWORK_FACTORY_DEFAULT_RESTORED = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FactoryEvent)))
  "Returns full string definition for message of type 'FactoryEvent"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_FACTORY_EVENT = 0~%~%uint32 FACTORY_DEFAULT_RESTORED = 1~%~%uint32 NETWORK_FACTORY_DEFAULT_RESTORED = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FactoryEvent>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FactoryEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'FactoryEvent
))
