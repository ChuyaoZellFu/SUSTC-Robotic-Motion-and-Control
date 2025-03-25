; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude UARTStopBits.msg.html

(cl:defclass <UARTStopBits> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass UARTStopBits (<UARTStopBits>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UARTStopBits>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UARTStopBits)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<UARTStopBits> is deprecated: use kortex_driver-msg:UARTStopBits instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<UARTStopBits>)))
    "Constants for message type '<UARTStopBits>"
  '((:UART_STOP_BITS_UNSPECIFIED . 0)
    (:UART_STOP_BITS_0_5 . 1)
    (:UART_STOP_BITS_1 . 2)
    (:UART_STOP_BITS_1_5 . 3)
    (:UART_STOP_BITS_2 . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'UARTStopBits)))
    "Constants for message type 'UARTStopBits"
  '((:UART_STOP_BITS_UNSPECIFIED . 0)
    (:UART_STOP_BITS_0_5 . 1)
    (:UART_STOP_BITS_1 . 2)
    (:UART_STOP_BITS_1_5 . 3)
    (:UART_STOP_BITS_2 . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UARTStopBits>) ostream)
  "Serializes a message object of type '<UARTStopBits>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UARTStopBits>) istream)
  "Deserializes a message object of type '<UARTStopBits>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UARTStopBits>)))
  "Returns string type for a message object of type '<UARTStopBits>"
  "kortex_driver/UARTStopBits")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UARTStopBits)))
  "Returns string type for a message object of type 'UARTStopBits"
  "kortex_driver/UARTStopBits")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UARTStopBits>)))
  "Returns md5sum for a message object of type '<UARTStopBits>"
  "41247c97dd0da3882638e6dea902eead")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UARTStopBits)))
  "Returns md5sum for a message object of type 'UARTStopBits"
  "41247c97dd0da3882638e6dea902eead")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UARTStopBits>)))
  "Returns full string definition for message of type '<UARTStopBits>"
  (cl:format cl:nil "~%uint32 UART_STOP_BITS_UNSPECIFIED = 0~%~%uint32 UART_STOP_BITS_0_5 = 1~%~%uint32 UART_STOP_BITS_1 = 2~%~%uint32 UART_STOP_BITS_1_5 = 3~%~%uint32 UART_STOP_BITS_2 = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UARTStopBits)))
  "Returns full string definition for message of type 'UARTStopBits"
  (cl:format cl:nil "~%uint32 UART_STOP_BITS_UNSPECIFIED = 0~%~%uint32 UART_STOP_BITS_0_5 = 1~%~%uint32 UART_STOP_BITS_1 = 2~%~%uint32 UART_STOP_BITS_1_5 = 3~%~%uint32 UART_STOP_BITS_2 = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UARTStopBits>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UARTStopBits>))
  "Converts a ROS message object to a list"
  (cl:list 'UARTStopBits
))
