; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude GpioAction.msg.html

(cl:defclass <GpioAction> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GpioAction (<GpioAction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GpioAction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GpioAction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<GpioAction> is deprecated: use kortex_driver-msg:GpioAction instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GpioAction>)))
    "Constants for message type '<GpioAction>"
  '((:UNSPECIFIED_GPIO_ACTION . 0)
    (:GPIOACTION_SET . 1)
    (:GPIOACTION_CLEAR . 2)
    (:GPIOACTION_PULSE_HIGH . 3)
    (:GPIOACTION_PULSE_LOW . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GpioAction)))
    "Constants for message type 'GpioAction"
  '((:UNSPECIFIED_GPIO_ACTION . 0)
    (:GPIOACTION_SET . 1)
    (:GPIOACTION_CLEAR . 2)
    (:GPIOACTION_PULSE_HIGH . 3)
    (:GPIOACTION_PULSE_LOW . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GpioAction>) ostream)
  "Serializes a message object of type '<GpioAction>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GpioAction>) istream)
  "Deserializes a message object of type '<GpioAction>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GpioAction>)))
  "Returns string type for a message object of type '<GpioAction>"
  "kortex_driver/GpioAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GpioAction)))
  "Returns string type for a message object of type 'GpioAction"
  "kortex_driver/GpioAction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GpioAction>)))
  "Returns md5sum for a message object of type '<GpioAction>"
  "7271e7564b2e393d951b0684902edec6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GpioAction)))
  "Returns md5sum for a message object of type 'GpioAction"
  "7271e7564b2e393d951b0684902edec6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GpioAction>)))
  "Returns full string definition for message of type '<GpioAction>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_GPIO_ACTION = 0~%~%uint32 GPIOACTION_SET = 1~%~%uint32 GPIOACTION_CLEAR = 2~%~%uint32 GPIOACTION_PULSE_HIGH = 3~%~%uint32 GPIOACTION_PULSE_LOW = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GpioAction)))
  "Returns full string definition for message of type 'GpioAction"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_GPIO_ACTION = 0~%~%uint32 GPIOACTION_SET = 1~%~%uint32 GPIOACTION_CLEAR = 2~%~%uint32 GPIOACTION_PULSE_HIGH = 3~%~%uint32 GPIOACTION_PULSE_LOW = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GpioAction>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GpioAction>))
  "Converts a ROS message object to a list"
  (cl:list 'GpioAction
))
