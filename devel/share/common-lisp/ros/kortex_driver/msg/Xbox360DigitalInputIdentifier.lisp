; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Xbox360DigitalInputIdentifier.msg.html

(cl:defclass <Xbox360DigitalInputIdentifier> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Xbox360DigitalInputIdentifier (<Xbox360DigitalInputIdentifier>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Xbox360DigitalInputIdentifier>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Xbox360DigitalInputIdentifier)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Xbox360DigitalInputIdentifier> is deprecated: use kortex_driver-msg:Xbox360DigitalInputIdentifier instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Xbox360DigitalInputIdentifier>)))
    "Constants for message type '<Xbox360DigitalInputIdentifier>"
  '((:UNSPECIFIED_XBOX360_DIGITAL . 0)
    (:XBOX360_PAD_UP . 1)
    (:XBOX360_PAD_DOWN . 2)
    (:XBOX360_PAD_LEFT . 3)
    (:XBOX360_PAD_RIGHT . 4)
    (:XBOX360_FILE_BUTTON_START . 5)
    (:XBOX360_DOCUMENT_BUTTON_BACK . 6)
    (:XBOX360_LEFT_THUMB_BUTTON . 7)
    (:XBOX360_RIGHT_THUMB_BUTTON . 8)
    (:XBOX360_LEFT_SHOULDER . 9)
    (:XBOX360_RIGHT_SHOULDER . 10)
    (:XBOX360_BUTTON_A . 13)
    (:XBOX360_BUTTON_B . 14)
    (:XBOX360_BUTTON_X . 15)
    (:XBOX360_BUTTON_Y . 16))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Xbox360DigitalInputIdentifier)))
    "Constants for message type 'Xbox360DigitalInputIdentifier"
  '((:UNSPECIFIED_XBOX360_DIGITAL . 0)
    (:XBOX360_PAD_UP . 1)
    (:XBOX360_PAD_DOWN . 2)
    (:XBOX360_PAD_LEFT . 3)
    (:XBOX360_PAD_RIGHT . 4)
    (:XBOX360_FILE_BUTTON_START . 5)
    (:XBOX360_DOCUMENT_BUTTON_BACK . 6)
    (:XBOX360_LEFT_THUMB_BUTTON . 7)
    (:XBOX360_RIGHT_THUMB_BUTTON . 8)
    (:XBOX360_LEFT_SHOULDER . 9)
    (:XBOX360_RIGHT_SHOULDER . 10)
    (:XBOX360_BUTTON_A . 13)
    (:XBOX360_BUTTON_B . 14)
    (:XBOX360_BUTTON_X . 15)
    (:XBOX360_BUTTON_Y . 16))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Xbox360DigitalInputIdentifier>) ostream)
  "Serializes a message object of type '<Xbox360DigitalInputIdentifier>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Xbox360DigitalInputIdentifier>) istream)
  "Deserializes a message object of type '<Xbox360DigitalInputIdentifier>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Xbox360DigitalInputIdentifier>)))
  "Returns string type for a message object of type '<Xbox360DigitalInputIdentifier>"
  "kortex_driver/Xbox360DigitalInputIdentifier")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Xbox360DigitalInputIdentifier)))
  "Returns string type for a message object of type 'Xbox360DigitalInputIdentifier"
  "kortex_driver/Xbox360DigitalInputIdentifier")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Xbox360DigitalInputIdentifier>)))
  "Returns md5sum for a message object of type '<Xbox360DigitalInputIdentifier>"
  "b1197219b8c5e86e78fa24aefadcb86e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Xbox360DigitalInputIdentifier)))
  "Returns md5sum for a message object of type 'Xbox360DigitalInputIdentifier"
  "b1197219b8c5e86e78fa24aefadcb86e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Xbox360DigitalInputIdentifier>)))
  "Returns full string definition for message of type '<Xbox360DigitalInputIdentifier>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_XBOX360_DIGITAL = 0~%~%uint32 XBOX360_PAD_UP = 1~%~%uint32 XBOX360_PAD_DOWN = 2~%~%uint32 XBOX360_PAD_LEFT = 3~%~%uint32 XBOX360_PAD_RIGHT = 4~%~%uint32 XBOX360_FILE_BUTTON_START = 5~%~%uint32 XBOX360_DOCUMENT_BUTTON_BACK = 6~%~%uint32 XBOX360_LEFT_THUMB_BUTTON = 7~%~%uint32 XBOX360_RIGHT_THUMB_BUTTON = 8~%~%uint32 XBOX360_LEFT_SHOULDER = 9~%~%uint32 XBOX360_RIGHT_SHOULDER = 10~%~%uint32 XBOX360_BUTTON_A = 13~%~%uint32 XBOX360_BUTTON_B = 14~%~%uint32 XBOX360_BUTTON_X = 15~%~%uint32 XBOX360_BUTTON_Y = 16~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Xbox360DigitalInputIdentifier)))
  "Returns full string definition for message of type 'Xbox360DigitalInputIdentifier"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_XBOX360_DIGITAL = 0~%~%uint32 XBOX360_PAD_UP = 1~%~%uint32 XBOX360_PAD_DOWN = 2~%~%uint32 XBOX360_PAD_LEFT = 3~%~%uint32 XBOX360_PAD_RIGHT = 4~%~%uint32 XBOX360_FILE_BUTTON_START = 5~%~%uint32 XBOX360_DOCUMENT_BUTTON_BACK = 6~%~%uint32 XBOX360_LEFT_THUMB_BUTTON = 7~%~%uint32 XBOX360_RIGHT_THUMB_BUTTON = 8~%~%uint32 XBOX360_LEFT_SHOULDER = 9~%~%uint32 XBOX360_RIGHT_SHOULDER = 10~%~%uint32 XBOX360_BUTTON_A = 13~%~%uint32 XBOX360_BUTTON_B = 14~%~%uint32 XBOX360_BUTTON_X = 15~%~%uint32 XBOX360_BUTTON_Y = 16~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Xbox360DigitalInputIdentifier>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Xbox360DigitalInputIdentifier>))
  "Converts a ROS message object to a list"
  (cl:list 'Xbox360DigitalInputIdentifier
))
