; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude VisionEvent.msg.html

(cl:defclass <VisionEvent> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass VisionEvent (<VisionEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisionEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisionEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<VisionEvent> is deprecated: use kortex_driver-msg:VisionEvent instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<VisionEvent>)))
    "Constants for message type '<VisionEvent>"
  '((:UNSPECIFIED_VISION_EVENT . 0)
    (:SENSOR_SETTINGS_CHANGED . 1)
    (:OPTION_VALUE_CHANGED . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'VisionEvent)))
    "Constants for message type 'VisionEvent"
  '((:UNSPECIFIED_VISION_EVENT . 0)
    (:SENSOR_SETTINGS_CHANGED . 1)
    (:OPTION_VALUE_CHANGED . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisionEvent>) ostream)
  "Serializes a message object of type '<VisionEvent>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisionEvent>) istream)
  "Deserializes a message object of type '<VisionEvent>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisionEvent>)))
  "Returns string type for a message object of type '<VisionEvent>"
  "kortex_driver/VisionEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisionEvent)))
  "Returns string type for a message object of type 'VisionEvent"
  "kortex_driver/VisionEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisionEvent>)))
  "Returns md5sum for a message object of type '<VisionEvent>"
  "5d538f4fa32289d8ec7691c8a8d47f4b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisionEvent)))
  "Returns md5sum for a message object of type 'VisionEvent"
  "5d538f4fa32289d8ec7691c8a8d47f4b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisionEvent>)))
  "Returns full string definition for message of type '<VisionEvent>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_VISION_EVENT = 0~%~%uint32 SENSOR_SETTINGS_CHANGED = 1~%~%uint32 OPTION_VALUE_CHANGED = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisionEvent)))
  "Returns full string definition for message of type 'VisionEvent"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_VISION_EVENT = 0~%~%uint32 SENSOR_SETTINGS_CHANGED = 1~%~%uint32 OPTION_VALUE_CHANGED = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisionEvent>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisionEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'VisionEvent
))
