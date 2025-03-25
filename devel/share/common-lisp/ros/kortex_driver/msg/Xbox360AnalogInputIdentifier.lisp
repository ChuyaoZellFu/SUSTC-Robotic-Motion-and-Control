; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Xbox360AnalogInputIdentifier.msg.html

(cl:defclass <Xbox360AnalogInputIdentifier> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Xbox360AnalogInputIdentifier (<Xbox360AnalogInputIdentifier>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Xbox360AnalogInputIdentifier>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Xbox360AnalogInputIdentifier)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Xbox360AnalogInputIdentifier> is deprecated: use kortex_driver-msg:Xbox360AnalogInputIdentifier instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Xbox360AnalogInputIdentifier>)))
    "Constants for message type '<Xbox360AnalogInputIdentifier>"
  '((:UNSPECIFIED_XBOX360_ANALOG . 0)
    (:XBOX360_THUMB_LEFT_X . 1)
    (:XBOX360_THUMB_LEFT_Y . 2)
    (:XBOX360_THUMB_RIGHT_X . 3)
    (:XBOX360_THUMB_RIGHT_Y . 4)
    (:XBOX360_TRIGGER_LEFT . 5)
    (:XBOX360_TRIGGER_RIGHT . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Xbox360AnalogInputIdentifier)))
    "Constants for message type 'Xbox360AnalogInputIdentifier"
  '((:UNSPECIFIED_XBOX360_ANALOG . 0)
    (:XBOX360_THUMB_LEFT_X . 1)
    (:XBOX360_THUMB_LEFT_Y . 2)
    (:XBOX360_THUMB_RIGHT_X . 3)
    (:XBOX360_THUMB_RIGHT_Y . 4)
    (:XBOX360_TRIGGER_LEFT . 5)
    (:XBOX360_TRIGGER_RIGHT . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Xbox360AnalogInputIdentifier>) ostream)
  "Serializes a message object of type '<Xbox360AnalogInputIdentifier>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Xbox360AnalogInputIdentifier>) istream)
  "Deserializes a message object of type '<Xbox360AnalogInputIdentifier>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Xbox360AnalogInputIdentifier>)))
  "Returns string type for a message object of type '<Xbox360AnalogInputIdentifier>"
  "kortex_driver/Xbox360AnalogInputIdentifier")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Xbox360AnalogInputIdentifier)))
  "Returns string type for a message object of type 'Xbox360AnalogInputIdentifier"
  "kortex_driver/Xbox360AnalogInputIdentifier")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Xbox360AnalogInputIdentifier>)))
  "Returns md5sum for a message object of type '<Xbox360AnalogInputIdentifier>"
  "bdf16a9285171f4643549a6ec207283d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Xbox360AnalogInputIdentifier)))
  "Returns md5sum for a message object of type 'Xbox360AnalogInputIdentifier"
  "bdf16a9285171f4643549a6ec207283d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Xbox360AnalogInputIdentifier>)))
  "Returns full string definition for message of type '<Xbox360AnalogInputIdentifier>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_XBOX360_ANALOG = 0~%~%uint32 XBOX360_THUMB_LEFT_X = 1~%~%uint32 XBOX360_THUMB_LEFT_Y = 2~%~%uint32 XBOX360_THUMB_RIGHT_X = 3~%~%uint32 XBOX360_THUMB_RIGHT_Y = 4~%~%uint32 XBOX360_TRIGGER_LEFT = 5~%~%uint32 XBOX360_TRIGGER_RIGHT = 6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Xbox360AnalogInputIdentifier)))
  "Returns full string definition for message of type 'Xbox360AnalogInputIdentifier"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_XBOX360_ANALOG = 0~%~%uint32 XBOX360_THUMB_LEFT_X = 1~%~%uint32 XBOX360_THUMB_LEFT_Y = 2~%~%uint32 XBOX360_THUMB_RIGHT_X = 3~%~%uint32 XBOX360_THUMB_RIGHT_Y = 4~%~%uint32 XBOX360_TRIGGER_LEFT = 5~%~%uint32 XBOX360_TRIGGER_RIGHT = 6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Xbox360AnalogInputIdentifier>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Xbox360AnalogInputIdentifier>))
  "Converts a ROS message object to a list"
  (cl:list 'Xbox360AnalogInputIdentifier
))
