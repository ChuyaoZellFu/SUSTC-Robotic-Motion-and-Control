; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SignalQuality.msg.html

(cl:defclass <SignalQuality> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SignalQuality (<SignalQuality>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SignalQuality>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SignalQuality)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SignalQuality> is deprecated: use kortex_driver-msg:SignalQuality instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SignalQuality>)))
    "Constants for message type '<SignalQuality>"
  '((:UNSPECIFIED_SIGNAL_QUALITY . 0)
    (:POOR . 1)
    (:FAIR . 2)
    (:GOOD . 3)
    (:EXCELLENT . 4)
    (:NONE . 5))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SignalQuality)))
    "Constants for message type 'SignalQuality"
  '((:UNSPECIFIED_SIGNAL_QUALITY . 0)
    (:POOR . 1)
    (:FAIR . 2)
    (:GOOD . 3)
    (:EXCELLENT . 4)
    (:NONE . 5))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SignalQuality>) ostream)
  "Serializes a message object of type '<SignalQuality>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SignalQuality>) istream)
  "Deserializes a message object of type '<SignalQuality>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SignalQuality>)))
  "Returns string type for a message object of type '<SignalQuality>"
  "kortex_driver/SignalQuality")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SignalQuality)))
  "Returns string type for a message object of type 'SignalQuality"
  "kortex_driver/SignalQuality")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SignalQuality>)))
  "Returns md5sum for a message object of type '<SignalQuality>"
  "080b7090353f86dd3efade43f40ad044")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SignalQuality)))
  "Returns md5sum for a message object of type 'SignalQuality"
  "080b7090353f86dd3efade43f40ad044")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SignalQuality>)))
  "Returns full string definition for message of type '<SignalQuality>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_SIGNAL_QUALITY = 0~%~%uint32 POOR = 1~%~%uint32 FAIR = 2~%~%uint32 GOOD = 3~%~%uint32 EXCELLENT = 4~%~%uint32 NONE = 5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SignalQuality)))
  "Returns full string definition for message of type 'SignalQuality"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_SIGNAL_QUALITY = 0~%~%uint32 POOR = 1~%~%uint32 FAIR = 2~%~%uint32 GOOD = 3~%~%uint32 EXCELLENT = 4~%~%uint32 NONE = 5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SignalQuality>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SignalQuality>))
  "Converts a ROS message object to a list"
  (cl:list 'SignalQuality
))
