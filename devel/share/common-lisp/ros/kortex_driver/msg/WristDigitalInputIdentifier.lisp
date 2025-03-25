; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude WristDigitalInputIdentifier.msg.html

(cl:defclass <WristDigitalInputIdentifier> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass WristDigitalInputIdentifier (<WristDigitalInputIdentifier>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WristDigitalInputIdentifier>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WristDigitalInputIdentifier)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<WristDigitalInputIdentifier> is deprecated: use kortex_driver-msg:WristDigitalInputIdentifier instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<WristDigitalInputIdentifier>)))
    "Constants for message type '<WristDigitalInputIdentifier>"
  '((:UNSPECIFIED_WRIST_DIGITAL . 0)
    (:WRIST_BUTTON_1 . 1)
    (:WRIST_BUTTON_2 . 2)
    (:WRIST_BUTTON_BOTH . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'WristDigitalInputIdentifier)))
    "Constants for message type 'WristDigitalInputIdentifier"
  '((:UNSPECIFIED_WRIST_DIGITAL . 0)
    (:WRIST_BUTTON_1 . 1)
    (:WRIST_BUTTON_2 . 2)
    (:WRIST_BUTTON_BOTH . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WristDigitalInputIdentifier>) ostream)
  "Serializes a message object of type '<WristDigitalInputIdentifier>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WristDigitalInputIdentifier>) istream)
  "Deserializes a message object of type '<WristDigitalInputIdentifier>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WristDigitalInputIdentifier>)))
  "Returns string type for a message object of type '<WristDigitalInputIdentifier>"
  "kortex_driver/WristDigitalInputIdentifier")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WristDigitalInputIdentifier)))
  "Returns string type for a message object of type 'WristDigitalInputIdentifier"
  "kortex_driver/WristDigitalInputIdentifier")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WristDigitalInputIdentifier>)))
  "Returns md5sum for a message object of type '<WristDigitalInputIdentifier>"
  "1ca46a2da6d637ba3dfa3b30f7d69da3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WristDigitalInputIdentifier)))
  "Returns md5sum for a message object of type 'WristDigitalInputIdentifier"
  "1ca46a2da6d637ba3dfa3b30f7d69da3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WristDigitalInputIdentifier>)))
  "Returns full string definition for message of type '<WristDigitalInputIdentifier>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_WRIST_DIGITAL = 0~%~%uint32 WRIST_BUTTON_1 = 1~%~%uint32 WRIST_BUTTON_2 = 2~%~%uint32 WRIST_BUTTON_BOTH = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WristDigitalInputIdentifier)))
  "Returns full string definition for message of type 'WristDigitalInputIdentifier"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_WRIST_DIGITAL = 0~%~%uint32 WRIST_BUTTON_1 = 1~%~%uint32 WRIST_BUTTON_2 = 2~%~%uint32 WRIST_BUTTON_BOTH = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WristDigitalInputIdentifier>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WristDigitalInputIdentifier>))
  "Converts a ROS message object to a list"
  (cl:list 'WristDigitalInputIdentifier
))
