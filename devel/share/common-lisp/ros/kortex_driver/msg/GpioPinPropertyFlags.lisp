; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude GpioPinPropertyFlags.msg.html

(cl:defclass <GpioPinPropertyFlags> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GpioPinPropertyFlags (<GpioPinPropertyFlags>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GpioPinPropertyFlags>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GpioPinPropertyFlags)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<GpioPinPropertyFlags> is deprecated: use kortex_driver-msg:GpioPinPropertyFlags instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GpioPinPropertyFlags>)))
    "Constants for message type '<GpioPinPropertyFlags>"
  '((:GPIOPROPERTY_UNKNOWN . 0)
    (:GPIOPROPERTY_INPUT . 1)
    (:GPIOPROPERTY_OUTPUT . 2)
    (:GPIOPROPERTY_ANALOG . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GpioPinPropertyFlags)))
    "Constants for message type 'GpioPinPropertyFlags"
  '((:GPIOPROPERTY_UNKNOWN . 0)
    (:GPIOPROPERTY_INPUT . 1)
    (:GPIOPROPERTY_OUTPUT . 2)
    (:GPIOPROPERTY_ANALOG . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GpioPinPropertyFlags>) ostream)
  "Serializes a message object of type '<GpioPinPropertyFlags>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GpioPinPropertyFlags>) istream)
  "Deserializes a message object of type '<GpioPinPropertyFlags>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GpioPinPropertyFlags>)))
  "Returns string type for a message object of type '<GpioPinPropertyFlags>"
  "kortex_driver/GpioPinPropertyFlags")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GpioPinPropertyFlags)))
  "Returns string type for a message object of type 'GpioPinPropertyFlags"
  "kortex_driver/GpioPinPropertyFlags")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GpioPinPropertyFlags>)))
  "Returns md5sum for a message object of type '<GpioPinPropertyFlags>"
  "531958ae411036543a3b84e9b7f802d3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GpioPinPropertyFlags)))
  "Returns md5sum for a message object of type 'GpioPinPropertyFlags"
  "531958ae411036543a3b84e9b7f802d3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GpioPinPropertyFlags>)))
  "Returns full string definition for message of type '<GpioPinPropertyFlags>"
  (cl:format cl:nil "~%uint32 GPIOPROPERTY_UNKNOWN = 0~%~%uint32 GPIOPROPERTY_INPUT = 1~%~%uint32 GPIOPROPERTY_OUTPUT = 2~%~%uint32 GPIOPROPERTY_ANALOG = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GpioPinPropertyFlags)))
  "Returns full string definition for message of type 'GpioPinPropertyFlags"
  (cl:format cl:nil "~%uint32 GPIOPROPERTY_UNKNOWN = 0~%~%uint32 GPIOPROPERTY_INPUT = 1~%~%uint32 GPIOPROPERTY_OUTPUT = 2~%~%uint32 GPIOPROPERTY_ANALOG = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GpioPinPropertyFlags>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GpioPinPropertyFlags>))
  "Converts a ROS message object to a list"
  (cl:list 'GpioPinPropertyFlags
))
