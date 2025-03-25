; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude FocusAction.msg.html

(cl:defclass <FocusAction> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass FocusAction (<FocusAction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FocusAction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FocusAction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<FocusAction> is deprecated: use kortex_driver-msg:FocusAction instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<FocusAction>)))
    "Constants for message type '<FocusAction>"
  '((:FOCUSACTION_UNSPECIFIED . 0)
    (:FOCUSACTION_START_CONTINUOUS_FOCUS . 1)
    (:FOCUSACTION_PAUSE_CONTINUOUS_FOCUS . 2)
    (:FOCUSACTION_FOCUS_NOW . 3)
    (:FOCUSACTION_DISABLE_FOCUS . 4)
    (:FOCUSACTION_SET_FOCUS_POINT . 5)
    (:FOCUSACTION_SET_MANUAL_FOCUS . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'FocusAction)))
    "Constants for message type 'FocusAction"
  '((:FOCUSACTION_UNSPECIFIED . 0)
    (:FOCUSACTION_START_CONTINUOUS_FOCUS . 1)
    (:FOCUSACTION_PAUSE_CONTINUOUS_FOCUS . 2)
    (:FOCUSACTION_FOCUS_NOW . 3)
    (:FOCUSACTION_DISABLE_FOCUS . 4)
    (:FOCUSACTION_SET_FOCUS_POINT . 5)
    (:FOCUSACTION_SET_MANUAL_FOCUS . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FocusAction>) ostream)
  "Serializes a message object of type '<FocusAction>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FocusAction>) istream)
  "Deserializes a message object of type '<FocusAction>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FocusAction>)))
  "Returns string type for a message object of type '<FocusAction>"
  "kortex_driver/FocusAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FocusAction)))
  "Returns string type for a message object of type 'FocusAction"
  "kortex_driver/FocusAction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FocusAction>)))
  "Returns md5sum for a message object of type '<FocusAction>"
  "16232e452a5de2238b9046f807acd748")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FocusAction)))
  "Returns md5sum for a message object of type 'FocusAction"
  "16232e452a5de2238b9046f807acd748")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FocusAction>)))
  "Returns full string definition for message of type '<FocusAction>"
  (cl:format cl:nil "~%uint32 FOCUSACTION_UNSPECIFIED = 0~%~%uint32 FOCUSACTION_START_CONTINUOUS_FOCUS = 1~%~%uint32 FOCUSACTION_PAUSE_CONTINUOUS_FOCUS = 2~%~%uint32 FOCUSACTION_FOCUS_NOW = 3~%~%uint32 FOCUSACTION_DISABLE_FOCUS = 4~%~%uint32 FOCUSACTION_SET_FOCUS_POINT = 5~%~%uint32 FOCUSACTION_SET_MANUAL_FOCUS = 6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FocusAction)))
  "Returns full string definition for message of type 'FocusAction"
  (cl:format cl:nil "~%uint32 FOCUSACTION_UNSPECIFIED = 0~%~%uint32 FOCUSACTION_START_CONTINUOUS_FOCUS = 1~%~%uint32 FOCUSACTION_PAUSE_CONTINUOUS_FOCUS = 2~%~%uint32 FOCUSACTION_FOCUS_NOW = 3~%~%uint32 FOCUSACTION_DISABLE_FOCUS = 4~%~%uint32 FOCUSACTION_SET_FOCUS_POINT = 5~%~%uint32 FOCUSACTION_SET_MANUAL_FOCUS = 6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FocusAction>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FocusAction>))
  "Converts a ROS message object to a list"
  (cl:list 'FocusAction
))
