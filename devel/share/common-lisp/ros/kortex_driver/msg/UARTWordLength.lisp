; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude UARTWordLength.msg.html

(cl:defclass <UARTWordLength> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass UARTWordLength (<UARTWordLength>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UARTWordLength>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UARTWordLength)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<UARTWordLength> is deprecated: use kortex_driver-msg:UARTWordLength instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<UARTWordLength>)))
    "Constants for message type '<UARTWordLength>"
  '((:UART_WORD_LENGTH_UNSPECIFIED . 0)
    (:UART_WORD_LENGTH_7 . 1)
    (:UART_WORD_LENGTH_8 . 2)
    (:UART_WORD_LENGTH_9 . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'UARTWordLength)))
    "Constants for message type 'UARTWordLength"
  '((:UART_WORD_LENGTH_UNSPECIFIED . 0)
    (:UART_WORD_LENGTH_7 . 1)
    (:UART_WORD_LENGTH_8 . 2)
    (:UART_WORD_LENGTH_9 . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UARTWordLength>) ostream)
  "Serializes a message object of type '<UARTWordLength>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UARTWordLength>) istream)
  "Deserializes a message object of type '<UARTWordLength>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UARTWordLength>)))
  "Returns string type for a message object of type '<UARTWordLength>"
  "kortex_driver/UARTWordLength")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UARTWordLength)))
  "Returns string type for a message object of type 'UARTWordLength"
  "kortex_driver/UARTWordLength")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UARTWordLength>)))
  "Returns md5sum for a message object of type '<UARTWordLength>"
  "d77bc1cf43de7d8c64548008ccbf4e86")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UARTWordLength)))
  "Returns md5sum for a message object of type 'UARTWordLength"
  "d77bc1cf43de7d8c64548008ccbf4e86")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UARTWordLength>)))
  "Returns full string definition for message of type '<UARTWordLength>"
  (cl:format cl:nil "~%uint32 UART_WORD_LENGTH_UNSPECIFIED = 0~%~%uint32 UART_WORD_LENGTH_7 = 1~%~%uint32 UART_WORD_LENGTH_8 = 2~%~%uint32 UART_WORD_LENGTH_9 = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UARTWordLength)))
  "Returns full string definition for message of type 'UARTWordLength"
  (cl:format cl:nil "~%uint32 UART_WORD_LENGTH_UNSPECIFIED = 0~%~%uint32 UART_WORD_LENGTH_7 = 1~%~%uint32 UART_WORD_LENGTH_8 = 2~%~%uint32 UART_WORD_LENGTH_9 = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UARTWordLength>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UARTWordLength>))
  "Converts a ROS message object to a list"
  (cl:list 'UARTWordLength
))
