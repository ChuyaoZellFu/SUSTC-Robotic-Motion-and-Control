; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude UARTSpeed.msg.html

(cl:defclass <UARTSpeed> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass UARTSpeed (<UARTSpeed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UARTSpeed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UARTSpeed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<UARTSpeed> is deprecated: use kortex_driver-msg:UARTSpeed instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<UARTSpeed>)))
    "Constants for message type '<UARTSpeed>"
  '((:UART_SPEED_UNSPECIFIED . 0)
    (:UART_SPEED_4800 . 1)
    (:UART_SPEED_9600 . 2)
    (:UART_SPEED_19200 . 3)
    (:UART_SPEED_38400 . 4)
    (:UART_SPEED_57600 . 5)
    (:UART_SPEED_115200 . 6)
    (:UART_SPEED_230400 . 7)
    (:UART_SPEED_460800 . 8)
    (:UART_SPEED_921600 . 9)
    (:UART_SPEED_1382400 . 10)
    (:UART_SPEED_1612800 . 11)
    (:UART_SPEED_1843200 . 12)
    (:UART_SPEED_2073600 . 13)
    (:UART_SPEED_2188800 . 14)
    (:UART_SPEED_2246400 . 15))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'UARTSpeed)))
    "Constants for message type 'UARTSpeed"
  '((:UART_SPEED_UNSPECIFIED . 0)
    (:UART_SPEED_4800 . 1)
    (:UART_SPEED_9600 . 2)
    (:UART_SPEED_19200 . 3)
    (:UART_SPEED_38400 . 4)
    (:UART_SPEED_57600 . 5)
    (:UART_SPEED_115200 . 6)
    (:UART_SPEED_230400 . 7)
    (:UART_SPEED_460800 . 8)
    (:UART_SPEED_921600 . 9)
    (:UART_SPEED_1382400 . 10)
    (:UART_SPEED_1612800 . 11)
    (:UART_SPEED_1843200 . 12)
    (:UART_SPEED_2073600 . 13)
    (:UART_SPEED_2188800 . 14)
    (:UART_SPEED_2246400 . 15))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UARTSpeed>) ostream)
  "Serializes a message object of type '<UARTSpeed>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UARTSpeed>) istream)
  "Deserializes a message object of type '<UARTSpeed>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UARTSpeed>)))
  "Returns string type for a message object of type '<UARTSpeed>"
  "kortex_driver/UARTSpeed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UARTSpeed)))
  "Returns string type for a message object of type 'UARTSpeed"
  "kortex_driver/UARTSpeed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UARTSpeed>)))
  "Returns md5sum for a message object of type '<UARTSpeed>"
  "75fb3218a247fa4c93b3a9a066077af5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UARTSpeed)))
  "Returns md5sum for a message object of type 'UARTSpeed"
  "75fb3218a247fa4c93b3a9a066077af5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UARTSpeed>)))
  "Returns full string definition for message of type '<UARTSpeed>"
  (cl:format cl:nil "~%uint32 UART_SPEED_UNSPECIFIED = 0~%~%uint32 UART_SPEED_4800 = 1~%~%uint32 UART_SPEED_9600 = 2~%~%uint32 UART_SPEED_19200 = 3~%~%uint32 UART_SPEED_38400 = 4~%~%uint32 UART_SPEED_57600 = 5~%~%uint32 UART_SPEED_115200 = 6~%~%uint32 UART_SPEED_230400 = 7~%~%uint32 UART_SPEED_460800 = 8~%~%uint32 UART_SPEED_921600 = 9~%~%uint32 UART_SPEED_1382400 = 10~%~%uint32 UART_SPEED_1612800 = 11~%~%uint32 UART_SPEED_1843200 = 12~%~%uint32 UART_SPEED_2073600 = 13~%~%uint32 UART_SPEED_2188800 = 14~%~%uint32 UART_SPEED_2246400 = 15~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UARTSpeed)))
  "Returns full string definition for message of type 'UARTSpeed"
  (cl:format cl:nil "~%uint32 UART_SPEED_UNSPECIFIED = 0~%~%uint32 UART_SPEED_4800 = 1~%~%uint32 UART_SPEED_9600 = 2~%~%uint32 UART_SPEED_19200 = 3~%~%uint32 UART_SPEED_38400 = 4~%~%uint32 UART_SPEED_57600 = 5~%~%uint32 UART_SPEED_115200 = 6~%~%uint32 UART_SPEED_230400 = 7~%~%uint32 UART_SPEED_460800 = 8~%~%uint32 UART_SPEED_921600 = 9~%~%uint32 UART_SPEED_1382400 = 10~%~%uint32 UART_SPEED_1612800 = 11~%~%uint32 UART_SPEED_1843200 = 12~%~%uint32 UART_SPEED_2073600 = 13~%~%uint32 UART_SPEED_2188800 = 14~%~%uint32 UART_SPEED_2246400 = 15~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UARTSpeed>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UARTSpeed>))
  "Converts a ROS message object to a list"
  (cl:list 'UARTSpeed
))
