; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude JointNavigationDirection.msg.html

(cl:defclass <JointNavigationDirection> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass JointNavigationDirection (<JointNavigationDirection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointNavigationDirection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointNavigationDirection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<JointNavigationDirection> is deprecated: use kortex_driver-msg:JointNavigationDirection instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<JointNavigationDirection>)))
    "Constants for message type '<JointNavigationDirection>"
  '((:UNSPECIFIED_JOINT_NAVIGATION_DIRECTION . 0)
    (:JOINT_NEXT . 1)
    (:JOINT_PREVIOUS . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'JointNavigationDirection)))
    "Constants for message type 'JointNavigationDirection"
  '((:UNSPECIFIED_JOINT_NAVIGATION_DIRECTION . 0)
    (:JOINT_NEXT . 1)
    (:JOINT_PREVIOUS . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointNavigationDirection>) ostream)
  "Serializes a message object of type '<JointNavigationDirection>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointNavigationDirection>) istream)
  "Deserializes a message object of type '<JointNavigationDirection>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointNavigationDirection>)))
  "Returns string type for a message object of type '<JointNavigationDirection>"
  "kortex_driver/JointNavigationDirection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointNavigationDirection)))
  "Returns string type for a message object of type 'JointNavigationDirection"
  "kortex_driver/JointNavigationDirection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointNavigationDirection>)))
  "Returns md5sum for a message object of type '<JointNavigationDirection>"
  "718d5eb2fe311f7d85c6686a0a7a3c9a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointNavigationDirection)))
  "Returns md5sum for a message object of type 'JointNavigationDirection"
  "718d5eb2fe311f7d85c6686a0a7a3c9a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointNavigationDirection>)))
  "Returns full string definition for message of type '<JointNavigationDirection>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_JOINT_NAVIGATION_DIRECTION = 0~%~%uint32 JOINT_NEXT = 1~%~%uint32 JOINT_PREVIOUS = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointNavigationDirection)))
  "Returns full string definition for message of type 'JointNavigationDirection"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_JOINT_NAVIGATION_DIRECTION = 0~%~%uint32 JOINT_NEXT = 1~%~%uint32 JOINT_PREVIOUS = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointNavigationDirection>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointNavigationDirection>))
  "Converts a ROS message object to a list"
  (cl:list 'JointNavigationDirection
))
