; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CommandMode.msg.html

(cl:defclass <CommandMode> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CommandMode (<CommandMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommandMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommandMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CommandMode> is deprecated: use kortex_driver-msg:CommandMode instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CommandMode>)))
    "Constants for message type '<CommandMode>"
  '((:CYCLIC . 0)
    (:ASYNC_CYCLIC_FLAGS . 1)
    (:ASYNC . 2)
    (:CYCLIC_JITTERCOMPENSATED_POSITION . 3)
    (:CYCLIC_JITTERCOMPENSATED_VELOCITY . 4)
    (:CYCLIC_JITTERCOMPENSATED_ACCELERATION . 5))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CommandMode)))
    "Constants for message type 'CommandMode"
  '((:CYCLIC . 0)
    (:ASYNC_CYCLIC_FLAGS . 1)
    (:ASYNC . 2)
    (:CYCLIC_JITTERCOMPENSATED_POSITION . 3)
    (:CYCLIC_JITTERCOMPENSATED_VELOCITY . 4)
    (:CYCLIC_JITTERCOMPENSATED_ACCELERATION . 5))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommandMode>) ostream)
  "Serializes a message object of type '<CommandMode>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommandMode>) istream)
  "Deserializes a message object of type '<CommandMode>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommandMode>)))
  "Returns string type for a message object of type '<CommandMode>"
  "kortex_driver/CommandMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandMode)))
  "Returns string type for a message object of type 'CommandMode"
  "kortex_driver/CommandMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommandMode>)))
  "Returns md5sum for a message object of type '<CommandMode>"
  "2d888c2b80a47e457086f44a5dd3543e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommandMode)))
  "Returns md5sum for a message object of type 'CommandMode"
  "2d888c2b80a47e457086f44a5dd3543e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommandMode>)))
  "Returns full string definition for message of type '<CommandMode>"
  (cl:format cl:nil "~%uint32 CYCLIC = 0~%~%uint32 ASYNC_CYCLIC_FLAGS = 1~%~%uint32 ASYNC = 2~%~%uint32 CYCLIC_JITTERCOMPENSATED_POSITION = 3~%~%uint32 CYCLIC_JITTERCOMPENSATED_VELOCITY = 4~%~%uint32 CYCLIC_JITTERCOMPENSATED_ACCELERATION = 5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommandMode)))
  "Returns full string definition for message of type 'CommandMode"
  (cl:format cl:nil "~%uint32 CYCLIC = 0~%~%uint32 ASYNC_CYCLIC_FLAGS = 1~%~%uint32 ASYNC = 2~%~%uint32 CYCLIC_JITTERCOMPENSATED_POSITION = 3~%~%uint32 CYCLIC_JITTERCOMPENSATED_VELOCITY = 4~%~%uint32 CYCLIC_JITTERCOMPENSATED_ACCELERATION = 5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommandMode>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommandMode>))
  "Converts a ROS message object to a list"
  (cl:list 'CommandMode
))
