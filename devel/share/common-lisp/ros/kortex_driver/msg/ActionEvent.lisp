; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ActionEvent.msg.html

(cl:defclass <ActionEvent> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ActionEvent (<ActionEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActionEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActionEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ActionEvent> is deprecated: use kortex_driver-msg:ActionEvent instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ActionEvent>)))
    "Constants for message type '<ActionEvent>"
  '((:UNSPECIFIED_ACTION_EVENT . 0)
    (:ACTION_END . 1)
    (:ACTION_ABORT . 2)
    (:ACTION_PAUSE . 3)
    (:ACTION_START . 4)
    (:ACTION_PREPROCESS_START . 5)
    (:ACTION_PREPROCESS_ABORT . 6)
    (:ACTION_PREPROCESS_END . 7)
    (:ACTION_POSTPROCESS_START . 8)
    (:ACTION_POSTPROCESS_ABORT . 9)
    (:ACTION_POSTPROCESS_END . 10)
    (:ACTION_FEEDBACK . 11))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ActionEvent)))
    "Constants for message type 'ActionEvent"
  '((:UNSPECIFIED_ACTION_EVENT . 0)
    (:ACTION_END . 1)
    (:ACTION_ABORT . 2)
    (:ACTION_PAUSE . 3)
    (:ACTION_START . 4)
    (:ACTION_PREPROCESS_START . 5)
    (:ACTION_PREPROCESS_ABORT . 6)
    (:ACTION_PREPROCESS_END . 7)
    (:ACTION_POSTPROCESS_START . 8)
    (:ACTION_POSTPROCESS_ABORT . 9)
    (:ACTION_POSTPROCESS_END . 10)
    (:ACTION_FEEDBACK . 11))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActionEvent>) ostream)
  "Serializes a message object of type '<ActionEvent>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActionEvent>) istream)
  "Deserializes a message object of type '<ActionEvent>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActionEvent>)))
  "Returns string type for a message object of type '<ActionEvent>"
  "kortex_driver/ActionEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActionEvent)))
  "Returns string type for a message object of type 'ActionEvent"
  "kortex_driver/ActionEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActionEvent>)))
  "Returns md5sum for a message object of type '<ActionEvent>"
  "34076dc876693564b13adf0d712f14bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActionEvent)))
  "Returns md5sum for a message object of type 'ActionEvent"
  "34076dc876693564b13adf0d712f14bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActionEvent>)))
  "Returns full string definition for message of type '<ActionEvent>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_ACTION_EVENT = 0~%~%uint32 ACTION_END = 1~%~%uint32 ACTION_ABORT = 2~%~%uint32 ACTION_PAUSE = 3~%~%uint32 ACTION_START = 4~%~%uint32 ACTION_PREPROCESS_START = 5~%~%uint32 ACTION_PREPROCESS_ABORT = 6~%~%uint32 ACTION_PREPROCESS_END = 7~%~%uint32 ACTION_POSTPROCESS_START = 8~%~%uint32 ACTION_POSTPROCESS_ABORT = 9~%~%uint32 ACTION_POSTPROCESS_END = 10~%~%uint32 ACTION_FEEDBACK = 11~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActionEvent)))
  "Returns full string definition for message of type 'ActionEvent"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_ACTION_EVENT = 0~%~%uint32 ACTION_END = 1~%~%uint32 ACTION_ABORT = 2~%~%uint32 ACTION_PAUSE = 3~%~%uint32 ACTION_START = 4~%~%uint32 ACTION_PREPROCESS_START = 5~%~%uint32 ACTION_PREPROCESS_ABORT = 6~%~%uint32 ACTION_PREPROCESS_END = 7~%~%uint32 ACTION_POSTPROCESS_START = 8~%~%uint32 ACTION_POSTPROCESS_ABORT = 9~%~%uint32 ACTION_POSTPROCESS_END = 10~%~%uint32 ACTION_FEEDBACK = 11~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActionEvent>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActionEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'ActionEvent
))
