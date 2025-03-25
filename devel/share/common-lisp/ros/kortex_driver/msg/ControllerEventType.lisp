; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControllerEventType.msg.html

(cl:defclass <ControllerEventType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ControllerEventType (<ControllerEventType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerEventType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerEventType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControllerEventType> is deprecated: use kortex_driver-msg:ControllerEventType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ControllerEventType>)))
    "Constants for message type '<ControllerEventType>"
  '((:UNSPECIFIED_CONTROLLER_EVENT . 0)
    (:CONTROLLER_DISCONNECTED . 1)
    (:CONTROLLER_CONNECTED . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ControllerEventType)))
    "Constants for message type 'ControllerEventType"
  '((:UNSPECIFIED_CONTROLLER_EVENT . 0)
    (:CONTROLLER_DISCONNECTED . 1)
    (:CONTROLLER_CONNECTED . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerEventType>) ostream)
  "Serializes a message object of type '<ControllerEventType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerEventType>) istream)
  "Deserializes a message object of type '<ControllerEventType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerEventType>)))
  "Returns string type for a message object of type '<ControllerEventType>"
  "kortex_driver/ControllerEventType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerEventType)))
  "Returns string type for a message object of type 'ControllerEventType"
  "kortex_driver/ControllerEventType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerEventType>)))
  "Returns md5sum for a message object of type '<ControllerEventType>"
  "4bffb5259aaf49860456ce038da78405")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerEventType)))
  "Returns md5sum for a message object of type 'ControllerEventType"
  "4bffb5259aaf49860456ce038da78405")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerEventType>)))
  "Returns full string definition for message of type '<ControllerEventType>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CONTROLLER_EVENT = 0~%~%uint32 CONTROLLER_DISCONNECTED = 1~%~%uint32 CONTROLLER_CONNECTED = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerEventType)))
  "Returns full string definition for message of type 'ControllerEventType"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CONTROLLER_EVENT = 0~%~%uint32 CONTROLLER_DISCONNECTED = 1~%~%uint32 CONTROLLER_CONNECTED = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerEventType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerEventType>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerEventType
))
