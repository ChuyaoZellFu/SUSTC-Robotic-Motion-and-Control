; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude EndEffectorType.msg.html

(cl:defclass <EndEffectorType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass EndEffectorType (<EndEffectorType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EndEffectorType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EndEffectorType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<EndEffectorType> is deprecated: use kortex_driver-msg:EndEffectorType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<EndEffectorType>)))
    "Constants for message type '<EndEffectorType>"
  '((:END_EFFECTOR_TYPE_UNSPECIFIED . 0)
    (:END_EFFECTOR_TYPE_NOT_INSTALLED . 1)
    (:END_EFFECTOR_TYPE_L31_GRIPPER_2F . 2)
    (:END_EFFECTOR_TYPE_ROBOTIQ_2F_85 . 3)
    (:END_EFFECTOR_TYPE_ROBOTIQ_2F_140 . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'EndEffectorType)))
    "Constants for message type 'EndEffectorType"
  '((:END_EFFECTOR_TYPE_UNSPECIFIED . 0)
    (:END_EFFECTOR_TYPE_NOT_INSTALLED . 1)
    (:END_EFFECTOR_TYPE_L31_GRIPPER_2F . 2)
    (:END_EFFECTOR_TYPE_ROBOTIQ_2F_85 . 3)
    (:END_EFFECTOR_TYPE_ROBOTIQ_2F_140 . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EndEffectorType>) ostream)
  "Serializes a message object of type '<EndEffectorType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EndEffectorType>) istream)
  "Deserializes a message object of type '<EndEffectorType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EndEffectorType>)))
  "Returns string type for a message object of type '<EndEffectorType>"
  "kortex_driver/EndEffectorType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EndEffectorType)))
  "Returns string type for a message object of type 'EndEffectorType"
  "kortex_driver/EndEffectorType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EndEffectorType>)))
  "Returns md5sum for a message object of type '<EndEffectorType>"
  "93f96d9a78bf91aec505afd56097a885")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EndEffectorType)))
  "Returns md5sum for a message object of type 'EndEffectorType"
  "93f96d9a78bf91aec505afd56097a885")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EndEffectorType>)))
  "Returns full string definition for message of type '<EndEffectorType>"
  (cl:format cl:nil "~%uint32 END_EFFECTOR_TYPE_UNSPECIFIED = 0~%~%uint32 END_EFFECTOR_TYPE_NOT_INSTALLED = 1~%~%uint32 END_EFFECTOR_TYPE_L31_GRIPPER_2F = 2~%~%uint32 END_EFFECTOR_TYPE_ROBOTIQ_2F_85 = 3~%~%uint32 END_EFFECTOR_TYPE_ROBOTIQ_2F_140 = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EndEffectorType)))
  "Returns full string definition for message of type 'EndEffectorType"
  (cl:format cl:nil "~%uint32 END_EFFECTOR_TYPE_UNSPECIFIED = 0~%~%uint32 END_EFFECTOR_TYPE_NOT_INSTALLED = 1~%~%uint32 END_EFFECTOR_TYPE_L31_GRIPPER_2F = 2~%~%uint32 END_EFFECTOR_TYPE_ROBOTIQ_2F_85 = 3~%~%uint32 END_EFFECTOR_TYPE_ROBOTIQ_2F_140 = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EndEffectorType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EndEffectorType>))
  "Converts a ROS message object to a list"
  (cl:list 'EndEffectorType
))
