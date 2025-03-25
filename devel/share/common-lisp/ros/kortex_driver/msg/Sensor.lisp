; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Sensor.msg.html

(cl:defclass <Sensor> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Sensor (<Sensor>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Sensor>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Sensor)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Sensor> is deprecated: use kortex_driver-msg:Sensor instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Sensor>)))
    "Constants for message type '<Sensor>"
  '((:SENSOR_UNSPECIFIED . 0)
    (:SENSOR_COLOR . 1)
    (:SENSOR_DEPTH . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Sensor)))
    "Constants for message type 'Sensor"
  '((:SENSOR_UNSPECIFIED . 0)
    (:SENSOR_COLOR . 1)
    (:SENSOR_DEPTH . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Sensor>) ostream)
  "Serializes a message object of type '<Sensor>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Sensor>) istream)
  "Deserializes a message object of type '<Sensor>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Sensor>)))
  "Returns string type for a message object of type '<Sensor>"
  "kortex_driver/Sensor")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sensor)))
  "Returns string type for a message object of type 'Sensor"
  "kortex_driver/Sensor")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Sensor>)))
  "Returns md5sum for a message object of type '<Sensor>"
  "4dd10d615f1a44376aea67ca8eb604d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Sensor)))
  "Returns md5sum for a message object of type 'Sensor"
  "4dd10d615f1a44376aea67ca8eb604d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Sensor>)))
  "Returns full string definition for message of type '<Sensor>"
  (cl:format cl:nil "~%uint32 SENSOR_UNSPECIFIED = 0~%~%uint32 SENSOR_COLOR = 1~%~%uint32 SENSOR_DEPTH = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Sensor)))
  "Returns full string definition for message of type 'Sensor"
  (cl:format cl:nil "~%uint32 SENSOR_UNSPECIFIED = 0~%~%uint32 SENSOR_COLOR = 1~%~%uint32 SENSOR_DEPTH = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Sensor>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Sensor>))
  "Converts a ROS message object to a list"
  (cl:list 'Sensor
))
