; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SoundType.msg.html

(cl:defclass <SoundType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SoundType (<SoundType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SoundType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SoundType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SoundType> is deprecated: use kortex_driver-msg:SoundType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SoundType>)))
    "Constants for message type '<SoundType>"
  '((:UNSPECIFIED_SOUND_TYPE . 0)
    (:BIP_SERIES . 1)
    (:SINGLE_BIP . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SoundType)))
    "Constants for message type 'SoundType"
  '((:UNSPECIFIED_SOUND_TYPE . 0)
    (:BIP_SERIES . 1)
    (:SINGLE_BIP . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SoundType>) ostream)
  "Serializes a message object of type '<SoundType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SoundType>) istream)
  "Deserializes a message object of type '<SoundType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SoundType>)))
  "Returns string type for a message object of type '<SoundType>"
  "kortex_driver/SoundType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SoundType)))
  "Returns string type for a message object of type 'SoundType"
  "kortex_driver/SoundType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SoundType>)))
  "Returns md5sum for a message object of type '<SoundType>"
  "478cf44e3f93e67de58f0a6007a206fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SoundType)))
  "Returns md5sum for a message object of type 'SoundType"
  "478cf44e3f93e67de58f0a6007a206fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SoundType>)))
  "Returns full string definition for message of type '<SoundType>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_SOUND_TYPE = 0~%~%uint32 BIP_SERIES = 1~%~%uint32 SINGLE_BIP = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SoundType)))
  "Returns full string definition for message of type 'SoundType"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_SOUND_TYPE = 0~%~%uint32 BIP_SERIES = 1~%~%uint32 SINGLE_BIP = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SoundType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SoundType>))
  "Converts a ROS message object to a list"
  (cl:list 'SoundType
))
