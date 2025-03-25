; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude EmergencyStop.msg.html

(cl:defclass <EmergencyStop> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass EmergencyStop (<EmergencyStop>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EmergencyStop>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EmergencyStop)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<EmergencyStop> is deprecated: use kortex_driver-msg:EmergencyStop instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EmergencyStop>) ostream)
  "Serializes a message object of type '<EmergencyStop>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EmergencyStop>) istream)
  "Deserializes a message object of type '<EmergencyStop>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EmergencyStop>)))
  "Returns string type for a message object of type '<EmergencyStop>"
  "kortex_driver/EmergencyStop")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EmergencyStop)))
  "Returns string type for a message object of type 'EmergencyStop"
  "kortex_driver/EmergencyStop")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EmergencyStop>)))
  "Returns md5sum for a message object of type '<EmergencyStop>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EmergencyStop)))
  "Returns md5sum for a message object of type 'EmergencyStop"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EmergencyStop>)))
  "Returns full string definition for message of type '<EmergencyStop>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EmergencyStop)))
  "Returns full string definition for message of type 'EmergencyStop"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EmergencyStop>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EmergencyStop>))
  "Converts a ROS message object to a list"
  (cl:list 'EmergencyStop
))
