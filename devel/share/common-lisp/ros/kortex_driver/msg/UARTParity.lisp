; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude UARTParity.msg.html

(cl:defclass <UARTParity> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass UARTParity (<UARTParity>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UARTParity>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UARTParity)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<UARTParity> is deprecated: use kortex_driver-msg:UARTParity instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<UARTParity>)))
    "Constants for message type '<UARTParity>"
  '((:UART_PARITY_UNSPECIFIED . 0)
    (:UART_PARITY_NONE . 1)
    (:UART_PARITY_ODD . 2)
    (:UART_PARITY_EVEN . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'UARTParity)))
    "Constants for message type 'UARTParity"
  '((:UART_PARITY_UNSPECIFIED . 0)
    (:UART_PARITY_NONE . 1)
    (:UART_PARITY_ODD . 2)
    (:UART_PARITY_EVEN . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UARTParity>) ostream)
  "Serializes a message object of type '<UARTParity>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UARTParity>) istream)
  "Deserializes a message object of type '<UARTParity>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UARTParity>)))
  "Returns string type for a message object of type '<UARTParity>"
  "kortex_driver/UARTParity")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UARTParity)))
  "Returns string type for a message object of type 'UARTParity"
  "kortex_driver/UARTParity")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UARTParity>)))
  "Returns md5sum for a message object of type '<UARTParity>"
  "02ea0fa72b95d9977ccc489459fb1109")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UARTParity)))
  "Returns md5sum for a message object of type 'UARTParity"
  "02ea0fa72b95d9977ccc489459fb1109")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UARTParity>)))
  "Returns full string definition for message of type '<UARTParity>"
  (cl:format cl:nil "~%uint32 UART_PARITY_UNSPECIFIED = 0~%~%uint32 UART_PARITY_NONE = 1~%~%uint32 UART_PARITY_ODD = 2~%~%uint32 UART_PARITY_EVEN = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UARTParity)))
  "Returns full string definition for message of type 'UARTParity"
  (cl:format cl:nil "~%uint32 UART_PARITY_UNSPECIFIED = 0~%~%uint32 UART_PARITY_NONE = 1~%~%uint32 UART_PARITY_ODD = 2~%~%uint32 UART_PARITY_EVEN = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UARTParity>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UARTParity>))
  "Converts a ROS message object to a list"
  (cl:list 'UARTParity
))
