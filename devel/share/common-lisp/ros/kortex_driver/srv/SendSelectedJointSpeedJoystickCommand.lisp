; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SendSelectedJointSpeedJoystickCommand-request.msg.html

(cl:defclass <SendSelectedJointSpeedJoystickCommand-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:JointSpeed
    :initform (cl:make-instance 'kortex_driver-msg:JointSpeed)))
)

(cl:defclass SendSelectedJointSpeedJoystickCommand-request (<SendSelectedJointSpeedJoystickCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendSelectedJointSpeedJoystickCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendSelectedJointSpeedJoystickCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendSelectedJointSpeedJoystickCommand-request> is deprecated: use kortex_driver-srv:SendSelectedJointSpeedJoystickCommand-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SendSelectedJointSpeedJoystickCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendSelectedJointSpeedJoystickCommand-request>) ostream)
  "Serializes a message object of type '<SendSelectedJointSpeedJoystickCommand-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendSelectedJointSpeedJoystickCommand-request>) istream)
  "Deserializes a message object of type '<SendSelectedJointSpeedJoystickCommand-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendSelectedJointSpeedJoystickCommand-request>)))
  "Returns string type for a service object of type '<SendSelectedJointSpeedJoystickCommand-request>"
  "kortex_driver/SendSelectedJointSpeedJoystickCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendSelectedJointSpeedJoystickCommand-request)))
  "Returns string type for a service object of type 'SendSelectedJointSpeedJoystickCommand-request"
  "kortex_driver/SendSelectedJointSpeedJoystickCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendSelectedJointSpeedJoystickCommand-request>)))
  "Returns md5sum for a message object of type '<SendSelectedJointSpeedJoystickCommand-request>"
  "966eae21a761830257b0f20dfcd24c8e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendSelectedJointSpeedJoystickCommand-request)))
  "Returns md5sum for a message object of type 'SendSelectedJointSpeedJoystickCommand-request"
  "966eae21a761830257b0f20dfcd24c8e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendSelectedJointSpeedJoystickCommand-request>)))
  "Returns full string definition for message of type '<SendSelectedJointSpeedJoystickCommand-request>"
  (cl:format cl:nil "JointSpeed input~%~%================================================================================~%MSG: kortex_driver/JointSpeed~%~%uint32 joint_identifier~%float32 value~%uint32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendSelectedJointSpeedJoystickCommand-request)))
  "Returns full string definition for message of type 'SendSelectedJointSpeedJoystickCommand-request"
  (cl:format cl:nil "JointSpeed input~%~%================================================================================~%MSG: kortex_driver/JointSpeed~%~%uint32 joint_identifier~%float32 value~%uint32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendSelectedJointSpeedJoystickCommand-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendSelectedJointSpeedJoystickCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SendSelectedJointSpeedJoystickCommand-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SendSelectedJointSpeedJoystickCommand-response.msg.html

(cl:defclass <SendSelectedJointSpeedJoystickCommand-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SendSelectedJointSpeedJoystickCommand-response (<SendSelectedJointSpeedJoystickCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendSelectedJointSpeedJoystickCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendSelectedJointSpeedJoystickCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendSelectedJointSpeedJoystickCommand-response> is deprecated: use kortex_driver-srv:SendSelectedJointSpeedJoystickCommand-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SendSelectedJointSpeedJoystickCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendSelectedJointSpeedJoystickCommand-response>) ostream)
  "Serializes a message object of type '<SendSelectedJointSpeedJoystickCommand-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendSelectedJointSpeedJoystickCommand-response>) istream)
  "Deserializes a message object of type '<SendSelectedJointSpeedJoystickCommand-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendSelectedJointSpeedJoystickCommand-response>)))
  "Returns string type for a service object of type '<SendSelectedJointSpeedJoystickCommand-response>"
  "kortex_driver/SendSelectedJointSpeedJoystickCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendSelectedJointSpeedJoystickCommand-response)))
  "Returns string type for a service object of type 'SendSelectedJointSpeedJoystickCommand-response"
  "kortex_driver/SendSelectedJointSpeedJoystickCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendSelectedJointSpeedJoystickCommand-response>)))
  "Returns md5sum for a message object of type '<SendSelectedJointSpeedJoystickCommand-response>"
  "966eae21a761830257b0f20dfcd24c8e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendSelectedJointSpeedJoystickCommand-response)))
  "Returns md5sum for a message object of type 'SendSelectedJointSpeedJoystickCommand-response"
  "966eae21a761830257b0f20dfcd24c8e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendSelectedJointSpeedJoystickCommand-response>)))
  "Returns full string definition for message of type '<SendSelectedJointSpeedJoystickCommand-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendSelectedJointSpeedJoystickCommand-response)))
  "Returns full string definition for message of type 'SendSelectedJointSpeedJoystickCommand-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendSelectedJointSpeedJoystickCommand-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendSelectedJointSpeedJoystickCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SendSelectedJointSpeedJoystickCommand-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SendSelectedJointSpeedJoystickCommand)))
  'SendSelectedJointSpeedJoystickCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SendSelectedJointSpeedJoystickCommand)))
  'SendSelectedJointSpeedJoystickCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendSelectedJointSpeedJoystickCommand)))
  "Returns string type for a service object of type '<SendSelectedJointSpeedJoystickCommand>"
  "kortex_driver/SendSelectedJointSpeedJoystickCommand")