; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude BitRate.msg.html

(cl:defclass <BitRate> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass BitRate (<BitRate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BitRate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BitRate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<BitRate> is deprecated: use kortex_driver-msg:BitRate instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<BitRate>)))
    "Constants for message type '<BitRate>"
  '((:BITRATE_UNSPECIFIED . 0)
    (:BITRATE_10_MBPS . 1)
    (:BITRATE_15_MBPS . 2)
    (:BITRATE_20_MBPS . 3)
    (:BITRATE_25_MBPS . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'BitRate)))
    "Constants for message type 'BitRate"
  '((:BITRATE_UNSPECIFIED . 0)
    (:BITRATE_10_MBPS . 1)
    (:BITRATE_15_MBPS . 2)
    (:BITRATE_20_MBPS . 3)
    (:BITRATE_25_MBPS . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BitRate>) ostream)
  "Serializes a message object of type '<BitRate>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BitRate>) istream)
  "Deserializes a message object of type '<BitRate>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BitRate>)))
  "Returns string type for a message object of type '<BitRate>"
  "kortex_driver/BitRate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BitRate)))
  "Returns string type for a message object of type 'BitRate"
  "kortex_driver/BitRate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BitRate>)))
  "Returns md5sum for a message object of type '<BitRate>"
  "472a4e4218d43df9cfed86c61a2322e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BitRate)))
  "Returns md5sum for a message object of type 'BitRate"
  "472a4e4218d43df9cfed86c61a2322e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BitRate>)))
  "Returns full string definition for message of type '<BitRate>"
  (cl:format cl:nil "~%uint32 BITRATE_UNSPECIFIED = 0~%~%uint32 BITRATE_10_MBPS = 1~%~%uint32 BITRATE_15_MBPS = 2~%~%uint32 BITRATE_20_MBPS = 3~%~%uint32 BITRATE_25_MBPS = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BitRate)))
  "Returns full string definition for message of type 'BitRate"
  (cl:format cl:nil "~%uint32 BITRATE_UNSPECIFIED = 0~%~%uint32 BITRATE_10_MBPS = 1~%~%uint32 BITRATE_15_MBPS = 2~%~%uint32 BITRATE_20_MBPS = 3~%~%uint32 BITRATE_25_MBPS = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BitRate>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BitRate>))
  "Converts a ROS message object to a list"
  (cl:list 'BitRate
))
