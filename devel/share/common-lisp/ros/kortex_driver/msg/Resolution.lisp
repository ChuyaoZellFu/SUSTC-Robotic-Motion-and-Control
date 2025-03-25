; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Resolution.msg.html

(cl:defclass <Resolution> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Resolution (<Resolution>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Resolution>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Resolution)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Resolution> is deprecated: use kortex_driver-msg:Resolution instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Resolution>)))
    "Constants for message type '<Resolution>"
  '((:RESOLUTION_UNSPECIFIED . 0)
    (:RESOLUTION_320X240 . 1)
    (:RESOLUTION_424X240 . 2)
    (:RESOLUTION_480X270 . 3)
    (:RESOLUTION_640X480 . 4)
    (:RESOLUTION_1280X720 . 5)
    (:RESOLUTION_1920X1080 . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Resolution)))
    "Constants for message type 'Resolution"
  '((:RESOLUTION_UNSPECIFIED . 0)
    (:RESOLUTION_320X240 . 1)
    (:RESOLUTION_424X240 . 2)
    (:RESOLUTION_480X270 . 3)
    (:RESOLUTION_640X480 . 4)
    (:RESOLUTION_1280X720 . 5)
    (:RESOLUTION_1920X1080 . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Resolution>) ostream)
  "Serializes a message object of type '<Resolution>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Resolution>) istream)
  "Deserializes a message object of type '<Resolution>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Resolution>)))
  "Returns string type for a message object of type '<Resolution>"
  "kortex_driver/Resolution")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Resolution)))
  "Returns string type for a message object of type 'Resolution"
  "kortex_driver/Resolution")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Resolution>)))
  "Returns md5sum for a message object of type '<Resolution>"
  "aa3fc86ed5b2ab08e6ab3385eebb39de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Resolution)))
  "Returns md5sum for a message object of type 'Resolution"
  "aa3fc86ed5b2ab08e6ab3385eebb39de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Resolution>)))
  "Returns full string definition for message of type '<Resolution>"
  (cl:format cl:nil "~%uint32 RESOLUTION_UNSPECIFIED = 0~%~%uint32 RESOLUTION_320x240 = 1~%~%uint32 RESOLUTION_424x240 = 2~%~%uint32 RESOLUTION_480x270 = 3~%~%uint32 RESOLUTION_640x480 = 4~%~%uint32 RESOLUTION_1280x720 = 5~%~%uint32 RESOLUTION_1920x1080 = 6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Resolution)))
  "Returns full string definition for message of type 'Resolution"
  (cl:format cl:nil "~%uint32 RESOLUTION_UNSPECIFIED = 0~%~%uint32 RESOLUTION_320x240 = 1~%~%uint32 RESOLUTION_424x240 = 2~%~%uint32 RESOLUTION_480x270 = 3~%~%uint32 RESOLUTION_640x480 = 4~%~%uint32 RESOLUTION_1280x720 = 5~%~%uint32 RESOLUTION_1920x1080 = 6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Resolution>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Resolution>))
  "Converts a ROS message object to a list"
  (cl:list 'Resolution
))
