; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CoggingFeedforwardMode.msg.html

(cl:defclass <CoggingFeedforwardMode> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CoggingFeedforwardMode (<CoggingFeedforwardMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CoggingFeedforwardMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CoggingFeedforwardMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CoggingFeedforwardMode> is deprecated: use kortex_driver-msg:CoggingFeedforwardMode instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CoggingFeedforwardMode>)))
    "Constants for message type '<CoggingFeedforwardMode>"
  '((:FEEDFORWARD_OFF . 0)
    (:FEEDFORWARD_ADAPTIVE . 1)
    (:FEEDFORWARD_CALIBRATED . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CoggingFeedforwardMode)))
    "Constants for message type 'CoggingFeedforwardMode"
  '((:FEEDFORWARD_OFF . 0)
    (:FEEDFORWARD_ADAPTIVE . 1)
    (:FEEDFORWARD_CALIBRATED . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CoggingFeedforwardMode>) ostream)
  "Serializes a message object of type '<CoggingFeedforwardMode>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CoggingFeedforwardMode>) istream)
  "Deserializes a message object of type '<CoggingFeedforwardMode>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CoggingFeedforwardMode>)))
  "Returns string type for a message object of type '<CoggingFeedforwardMode>"
  "kortex_driver/CoggingFeedforwardMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CoggingFeedforwardMode)))
  "Returns string type for a message object of type 'CoggingFeedforwardMode"
  "kortex_driver/CoggingFeedforwardMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CoggingFeedforwardMode>)))
  "Returns md5sum for a message object of type '<CoggingFeedforwardMode>"
  "040e60ae33515a7c8082e377632750e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CoggingFeedforwardMode)))
  "Returns md5sum for a message object of type 'CoggingFeedforwardMode"
  "040e60ae33515a7c8082e377632750e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CoggingFeedforwardMode>)))
  "Returns full string definition for message of type '<CoggingFeedforwardMode>"
  (cl:format cl:nil "~%uint32 FEEDFORWARD_OFF = 0~%~%uint32 FEEDFORWARD_ADAPTIVE = 1~%~%uint32 FEEDFORWARD_CALIBRATED = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CoggingFeedforwardMode)))
  "Returns full string definition for message of type 'CoggingFeedforwardMode"
  (cl:format cl:nil "~%uint32 FEEDFORWARD_OFF = 0~%~%uint32 FEEDFORWARD_ADAPTIVE = 1~%~%uint32 FEEDFORWARD_CALIBRATED = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CoggingFeedforwardMode>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CoggingFeedforwardMode>))
  "Converts a ROS message object to a list"
  (cl:list 'CoggingFeedforwardMode
))
