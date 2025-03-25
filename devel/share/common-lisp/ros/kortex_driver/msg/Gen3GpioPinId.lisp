; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Gen3GpioPinId.msg.html

(cl:defclass <Gen3GpioPinId> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Gen3GpioPinId (<Gen3GpioPinId>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Gen3GpioPinId>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Gen3GpioPinId)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Gen3GpioPinId> is deprecated: use kortex_driver-msg:Gen3GpioPinId instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Gen3GpioPinId>)))
    "Constants for message type '<Gen3GpioPinId>"
  '((:UNSPECIFIED_PIN . 0)
    (:GPIO_PIN_B . 1)
    (:GPIO_PIN_C . 2)
    (:GPIO_PIN_D . 3)
    (:GPIO_PIN_E . 4)
    (:GPIO_PIN_G . 5)
    (:GPIO_PIN_H . 6)
    (:GPIO_PIN_I . 7)
    (:GPIO_PIN_K . 8)
    (:GPIO_PIN_N . 9)
    (:GPIO_PIN_O . 10)
    (:GPIO_PIN_S . 11)
    (:GPIO_PIN_T . 12))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Gen3GpioPinId)))
    "Constants for message type 'Gen3GpioPinId"
  '((:UNSPECIFIED_PIN . 0)
    (:GPIO_PIN_B . 1)
    (:GPIO_PIN_C . 2)
    (:GPIO_PIN_D . 3)
    (:GPIO_PIN_E . 4)
    (:GPIO_PIN_G . 5)
    (:GPIO_PIN_H . 6)
    (:GPIO_PIN_I . 7)
    (:GPIO_PIN_K . 8)
    (:GPIO_PIN_N . 9)
    (:GPIO_PIN_O . 10)
    (:GPIO_PIN_S . 11)
    (:GPIO_PIN_T . 12))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Gen3GpioPinId>) ostream)
  "Serializes a message object of type '<Gen3GpioPinId>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Gen3GpioPinId>) istream)
  "Deserializes a message object of type '<Gen3GpioPinId>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Gen3GpioPinId>)))
  "Returns string type for a message object of type '<Gen3GpioPinId>"
  "kortex_driver/Gen3GpioPinId")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Gen3GpioPinId)))
  "Returns string type for a message object of type 'Gen3GpioPinId"
  "kortex_driver/Gen3GpioPinId")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Gen3GpioPinId>)))
  "Returns md5sum for a message object of type '<Gen3GpioPinId>"
  "e4129f3451b6d28fcb9b382446ece1a9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Gen3GpioPinId)))
  "Returns md5sum for a message object of type 'Gen3GpioPinId"
  "e4129f3451b6d28fcb9b382446ece1a9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Gen3GpioPinId>)))
  "Returns full string definition for message of type '<Gen3GpioPinId>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_PIN = 0~%~%uint32 GPIO_PIN_B = 1~%~%uint32 GPIO_PIN_C = 2~%~%uint32 GPIO_PIN_D = 3~%~%uint32 GPIO_PIN_E = 4~%~%uint32 GPIO_PIN_G = 5~%~%uint32 GPIO_PIN_H = 6~%~%uint32 GPIO_PIN_I = 7~%~%uint32 GPIO_PIN_K = 8~%~%uint32 GPIO_PIN_N = 9~%~%uint32 GPIO_PIN_O = 10~%~%uint32 GPIO_PIN_S = 11~%~%uint32 GPIO_PIN_T = 12~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Gen3GpioPinId)))
  "Returns full string definition for message of type 'Gen3GpioPinId"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_PIN = 0~%~%uint32 GPIO_PIN_B = 1~%~%uint32 GPIO_PIN_C = 2~%~%uint32 GPIO_PIN_D = 3~%~%uint32 GPIO_PIN_E = 4~%~%uint32 GPIO_PIN_G = 5~%~%uint32 GPIO_PIN_H = 6~%~%uint32 GPIO_PIN_I = 7~%~%uint32 GPIO_PIN_K = 8~%~%uint32 GPIO_PIN_N = 9~%~%uint32 GPIO_PIN_O = 10~%~%uint32 GPIO_PIN_S = 11~%~%uint32 GPIO_PIN_T = 12~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Gen3GpioPinId>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Gen3GpioPinId>))
  "Converts a ROS message object to a list"
  (cl:list 'Gen3GpioPinId
))
