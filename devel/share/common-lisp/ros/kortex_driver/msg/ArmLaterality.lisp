; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ArmLaterality.msg.html

(cl:defclass <ArmLaterality> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ArmLaterality (<ArmLaterality>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmLaterality>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmLaterality)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ArmLaterality> is deprecated: use kortex_driver-msg:ArmLaterality instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ArmLaterality>)))
    "Constants for message type '<ArmLaterality>"
  '((:ARM_LATERALITY_UNSPECIFIED . 0)
    (:ARM_LATERALITY_NOT_APPLICABLE . 1)
    (:ARM_LATERALITY_LEFT . 2)
    (:ARM_LATERALITY_RIGHT . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ArmLaterality)))
    "Constants for message type 'ArmLaterality"
  '((:ARM_LATERALITY_UNSPECIFIED . 0)
    (:ARM_LATERALITY_NOT_APPLICABLE . 1)
    (:ARM_LATERALITY_LEFT . 2)
    (:ARM_LATERALITY_RIGHT . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmLaterality>) ostream)
  "Serializes a message object of type '<ArmLaterality>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmLaterality>) istream)
  "Deserializes a message object of type '<ArmLaterality>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmLaterality>)))
  "Returns string type for a message object of type '<ArmLaterality>"
  "kortex_driver/ArmLaterality")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmLaterality)))
  "Returns string type for a message object of type 'ArmLaterality"
  "kortex_driver/ArmLaterality")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmLaterality>)))
  "Returns md5sum for a message object of type '<ArmLaterality>"
  "e87e70b626397c379fd01cdd30b3b822")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmLaterality)))
  "Returns md5sum for a message object of type 'ArmLaterality"
  "e87e70b626397c379fd01cdd30b3b822")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmLaterality>)))
  "Returns full string definition for message of type '<ArmLaterality>"
  (cl:format cl:nil "~%uint32 ARM_LATERALITY_UNSPECIFIED = 0~%~%uint32 ARM_LATERALITY_NOT_APPLICABLE = 1~%~%uint32 ARM_LATERALITY_LEFT = 2~%~%uint32 ARM_LATERALITY_RIGHT = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmLaterality)))
  "Returns full string definition for message of type 'ArmLaterality"
  (cl:format cl:nil "~%uint32 ARM_LATERALITY_UNSPECIFIED = 0~%~%uint32 ARM_LATERALITY_NOT_APPLICABLE = 1~%~%uint32 ARM_LATERALITY_LEFT = 2~%~%uint32 ARM_LATERALITY_RIGHT = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmLaterality>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmLaterality>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmLaterality
))
