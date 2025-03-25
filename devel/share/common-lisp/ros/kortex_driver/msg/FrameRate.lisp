; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude FrameRate.msg.html

(cl:defclass <FrameRate> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass FrameRate (<FrameRate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FrameRate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FrameRate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<FrameRate> is deprecated: use kortex_driver-msg:FrameRate instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<FrameRate>)))
    "Constants for message type '<FrameRate>"
  '((:FRAMERATE_UNSPECIFIED . 0)
    (:FRAMERATE_6_FPS . 1)
    (:FRAMERATE_15_FPS . 2)
    (:FRAMERATE_30_FPS . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'FrameRate)))
    "Constants for message type 'FrameRate"
  '((:FRAMERATE_UNSPECIFIED . 0)
    (:FRAMERATE_6_FPS . 1)
    (:FRAMERATE_15_FPS . 2)
    (:FRAMERATE_30_FPS . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FrameRate>) ostream)
  "Serializes a message object of type '<FrameRate>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FrameRate>) istream)
  "Deserializes a message object of type '<FrameRate>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FrameRate>)))
  "Returns string type for a message object of type '<FrameRate>"
  "kortex_driver/FrameRate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FrameRate)))
  "Returns string type for a message object of type 'FrameRate"
  "kortex_driver/FrameRate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FrameRate>)))
  "Returns md5sum for a message object of type '<FrameRate>"
  "9400397c1a84f4718dc33b3b7f3b5df4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FrameRate)))
  "Returns md5sum for a message object of type 'FrameRate"
  "9400397c1a84f4718dc33b3b7f3b5df4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FrameRate>)))
  "Returns full string definition for message of type '<FrameRate>"
  (cl:format cl:nil "~%uint32 FRAMERATE_UNSPECIFIED = 0~%~%uint32 FRAMERATE_6_FPS = 1~%~%uint32 FRAMERATE_15_FPS = 2~%~%uint32 FRAMERATE_30_FPS = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FrameRate)))
  "Returns full string definition for message of type 'FrameRate"
  (cl:format cl:nil "~%uint32 FRAMERATE_UNSPECIFIED = 0~%~%uint32 FRAMERATE_6_FPS = 1~%~%uint32 FRAMERATE_15_FPS = 2~%~%uint32 FRAMERATE_30_FPS = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FrameRate>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FrameRate>))
  "Converts a ROS message object to a list"
  (cl:list 'FrameRate
))
