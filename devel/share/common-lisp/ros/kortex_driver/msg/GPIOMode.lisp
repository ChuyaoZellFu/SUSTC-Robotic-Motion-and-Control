; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude GPIOMode.msg.html

(cl:defclass <GPIOMode> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GPIOMode (<GPIOMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GPIOMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GPIOMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<GPIOMode> is deprecated: use kortex_driver-msg:GPIOMode instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GPIOMode>)))
    "Constants for message type '<GPIOMode>"
  '((:GPIO_MODE_UNSPECIFIED . 0)
    (:GPIO_MODE_INPUT_FLOATING . 1)
    (:GPIO_MODE_OUTPUT_PUSH_PULL . 2)
    (:GPIO_MODE_OUTPUT_OPEN_DRAIN . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GPIOMode)))
    "Constants for message type 'GPIOMode"
  '((:GPIO_MODE_UNSPECIFIED . 0)
    (:GPIO_MODE_INPUT_FLOATING . 1)
    (:GPIO_MODE_OUTPUT_PUSH_PULL . 2)
    (:GPIO_MODE_OUTPUT_OPEN_DRAIN . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GPIOMode>) ostream)
  "Serializes a message object of type '<GPIOMode>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GPIOMode>) istream)
  "Deserializes a message object of type '<GPIOMode>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GPIOMode>)))
  "Returns string type for a message object of type '<GPIOMode>"
  "kortex_driver/GPIOMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GPIOMode)))
  "Returns string type for a message object of type 'GPIOMode"
  "kortex_driver/GPIOMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GPIOMode>)))
  "Returns md5sum for a message object of type '<GPIOMode>"
  "67dcb56ddb7c7d3c3c961e401c8325a4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GPIOMode)))
  "Returns md5sum for a message object of type 'GPIOMode"
  "67dcb56ddb7c7d3c3c961e401c8325a4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GPIOMode>)))
  "Returns full string definition for message of type '<GPIOMode>"
  (cl:format cl:nil "~%uint32 GPIO_MODE_UNSPECIFIED = 0~%~%uint32 GPIO_MODE_INPUT_FLOATING = 1~%~%uint32 GPIO_MODE_OUTPUT_PUSH_PULL = 2~%~%uint32 GPIO_MODE_OUTPUT_OPEN_DRAIN = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GPIOMode)))
  "Returns full string definition for message of type 'GPIOMode"
  (cl:format cl:nil "~%uint32 GPIO_MODE_UNSPECIFIED = 0~%~%uint32 GPIO_MODE_INPUT_FLOATING = 1~%~%uint32 GPIO_MODE_OUTPUT_PUSH_PULL = 2~%~%uint32 GPIO_MODE_OUTPUT_OPEN_DRAIN = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GPIOMode>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GPIOMode>))
  "Converts a ROS message object to a list"
  (cl:list 'GPIOMode
))
