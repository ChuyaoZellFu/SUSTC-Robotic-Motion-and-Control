; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Base_Stop.msg.html

(cl:defclass <Base_Stop> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Base_Stop (<Base_Stop>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Base_Stop>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Base_Stop)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Base_Stop> is deprecated: use kortex_driver-msg:Base_Stop instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Base_Stop>) ostream)
  "Serializes a message object of type '<Base_Stop>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Base_Stop>) istream)
  "Deserializes a message object of type '<Base_Stop>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Base_Stop>)))
  "Returns string type for a message object of type '<Base_Stop>"
  "kortex_driver/Base_Stop")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_Stop)))
  "Returns string type for a message object of type 'Base_Stop"
  "kortex_driver/Base_Stop")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Base_Stop>)))
  "Returns md5sum for a message object of type '<Base_Stop>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Base_Stop)))
  "Returns md5sum for a message object of type 'Base_Stop"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Base_Stop>)))
  "Returns full string definition for message of type '<Base_Stop>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Base_Stop)))
  "Returns full string definition for message of type 'Base_Stop"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Base_Stop>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Base_Stop>))
  "Converts a ROS message object to a list"
  (cl:list 'Base_Stop
))
