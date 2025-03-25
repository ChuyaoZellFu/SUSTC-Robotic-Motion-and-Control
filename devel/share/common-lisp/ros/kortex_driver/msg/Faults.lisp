; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Faults.msg.html

(cl:defclass <Faults> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Faults (<Faults>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Faults>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Faults)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Faults> is deprecated: use kortex_driver-msg:Faults instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Faults>) ostream)
  "Serializes a message object of type '<Faults>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Faults>) istream)
  "Deserializes a message object of type '<Faults>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Faults>)))
  "Returns string type for a message object of type '<Faults>"
  "kortex_driver/Faults")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Faults)))
  "Returns string type for a message object of type 'Faults"
  "kortex_driver/Faults")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Faults>)))
  "Returns md5sum for a message object of type '<Faults>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Faults)))
  "Returns md5sum for a message object of type 'Faults"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Faults>)))
  "Returns full string definition for message of type '<Faults>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Faults)))
  "Returns full string definition for message of type 'Faults"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Faults>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Faults>))
  "Converts a ROS message object to a list"
  (cl:list 'Faults
))
