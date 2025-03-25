; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControllerInputType.msg.html

(cl:defclass <ControllerInputType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ControllerInputType (<ControllerInputType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerInputType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerInputType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControllerInputType> is deprecated: use kortex_driver-msg:ControllerInputType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ControllerInputType>)))
    "Constants for message type '<ControllerInputType>"
  '((:UNSPECIFIED_CONTROLLER_INPUT_TYPE . 0)
    (:ANALOG . 1)
    (:DIGITAL . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ControllerInputType)))
    "Constants for message type 'ControllerInputType"
  '((:UNSPECIFIED_CONTROLLER_INPUT_TYPE . 0)
    (:ANALOG . 1)
    (:DIGITAL . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerInputType>) ostream)
  "Serializes a message object of type '<ControllerInputType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerInputType>) istream)
  "Deserializes a message object of type '<ControllerInputType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerInputType>)))
  "Returns string type for a message object of type '<ControllerInputType>"
  "kortex_driver/ControllerInputType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerInputType)))
  "Returns string type for a message object of type 'ControllerInputType"
  "kortex_driver/ControllerInputType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerInputType>)))
  "Returns md5sum for a message object of type '<ControllerInputType>"
  "758c1628f9ac7ba8ba58d7882102ea8f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerInputType)))
  "Returns md5sum for a message object of type 'ControllerInputType"
  "758c1628f9ac7ba8ba58d7882102ea8f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerInputType>)))
  "Returns full string definition for message of type '<ControllerInputType>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CONTROLLER_INPUT_TYPE = 0~%~%uint32 ANALOG = 1~%~%uint32 DIGITAL = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerInputType)))
  "Returns full string definition for message of type 'ControllerInputType"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CONTROLLER_INPUT_TYPE = 0~%~%uint32 ANALOG = 1~%~%uint32 DIGITAL = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerInputType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerInputType>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerInputType
))
