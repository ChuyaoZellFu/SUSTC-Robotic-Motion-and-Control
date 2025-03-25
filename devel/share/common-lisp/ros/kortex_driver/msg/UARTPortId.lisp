; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude UARTPortId.msg.html

(cl:defclass <UARTPortId> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass UARTPortId (<UARTPortId>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UARTPortId>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UARTPortId)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<UARTPortId> is deprecated: use kortex_driver-msg:UARTPortId instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<UARTPortId>)))
    "Constants for message type '<UARTPortId>"
  '((:UART_PORT_UNSPECIFIED . 0)
    (:UART_PORT_EXPANSION . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'UARTPortId)))
    "Constants for message type 'UARTPortId"
  '((:UART_PORT_UNSPECIFIED . 0)
    (:UART_PORT_EXPANSION . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UARTPortId>) ostream)
  "Serializes a message object of type '<UARTPortId>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UARTPortId>) istream)
  "Deserializes a message object of type '<UARTPortId>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UARTPortId>)))
  "Returns string type for a message object of type '<UARTPortId>"
  "kortex_driver/UARTPortId")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UARTPortId)))
  "Returns string type for a message object of type 'UARTPortId"
  "kortex_driver/UARTPortId")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UARTPortId>)))
  "Returns md5sum for a message object of type '<UARTPortId>"
  "9298e5854915673894eb15fed41f2130")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UARTPortId)))
  "Returns md5sum for a message object of type 'UARTPortId"
  "9298e5854915673894eb15fed41f2130")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UARTPortId>)))
  "Returns full string definition for message of type '<UARTPortId>"
  (cl:format cl:nil "~%uint32 UART_PORT_UNSPECIFIED = 0~%~%uint32 UART_PORT_EXPANSION = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UARTPortId)))
  "Returns full string definition for message of type 'UARTPortId"
  (cl:format cl:nil "~%uint32 UART_PORT_UNSPECIFIED = 0~%~%uint32 UART_PORT_EXPANSION = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UARTPortId>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UARTPortId>))
  "Converts a ROS message object to a list"
  (cl:list 'UARTPortId
))
