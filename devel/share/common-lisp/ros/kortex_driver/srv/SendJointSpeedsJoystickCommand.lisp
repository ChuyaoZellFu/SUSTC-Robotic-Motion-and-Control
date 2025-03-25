; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SendJointSpeedsJoystickCommand-request.msg.html

(cl:defclass <SendJointSpeedsJoystickCommand-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Base_JointSpeeds
    :initform (cl:make-instance 'kortex_driver-msg:Base_JointSpeeds)))
)

(cl:defclass SendJointSpeedsJoystickCommand-request (<SendJointSpeedsJoystickCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendJointSpeedsJoystickCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendJointSpeedsJoystickCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendJointSpeedsJoystickCommand-request> is deprecated: use kortex_driver-srv:SendJointSpeedsJoystickCommand-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SendJointSpeedsJoystickCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendJointSpeedsJoystickCommand-request>) ostream)
  "Serializes a message object of type '<SendJointSpeedsJoystickCommand-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendJointSpeedsJoystickCommand-request>) istream)
  "Deserializes a message object of type '<SendJointSpeedsJoystickCommand-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendJointSpeedsJoystickCommand-request>)))
  "Returns string type for a service object of type '<SendJointSpeedsJoystickCommand-request>"
  "kortex_driver/SendJointSpeedsJoystickCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendJointSpeedsJoystickCommand-request)))
  "Returns string type for a service object of type 'SendJointSpeedsJoystickCommand-request"
  "kortex_driver/SendJointSpeedsJoystickCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendJointSpeedsJoystickCommand-request>)))
  "Returns md5sum for a message object of type '<SendJointSpeedsJoystickCommand-request>"
  "35bff15135e19b4099e6a92d5e7d08d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendJointSpeedsJoystickCommand-request)))
  "Returns md5sum for a message object of type 'SendJointSpeedsJoystickCommand-request"
  "35bff15135e19b4099e6a92d5e7d08d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendJointSpeedsJoystickCommand-request>)))
  "Returns full string definition for message of type '<SendJointSpeedsJoystickCommand-request>"
  (cl:format cl:nil "Base_JointSpeeds input~%~%================================================================================~%MSG: kortex_driver/Base_JointSpeeds~%~%JointSpeed[] joint_speeds~%uint32 duration~%================================================================================~%MSG: kortex_driver/JointSpeed~%~%uint32 joint_identifier~%float32 value~%uint32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendJointSpeedsJoystickCommand-request)))
  "Returns full string definition for message of type 'SendJointSpeedsJoystickCommand-request"
  (cl:format cl:nil "Base_JointSpeeds input~%~%================================================================================~%MSG: kortex_driver/Base_JointSpeeds~%~%JointSpeed[] joint_speeds~%uint32 duration~%================================================================================~%MSG: kortex_driver/JointSpeed~%~%uint32 joint_identifier~%float32 value~%uint32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendJointSpeedsJoystickCommand-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendJointSpeedsJoystickCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SendJointSpeedsJoystickCommand-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SendJointSpeedsJoystickCommand-response.msg.html

(cl:defclass <SendJointSpeedsJoystickCommand-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SendJointSpeedsJoystickCommand-response (<SendJointSpeedsJoystickCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendJointSpeedsJoystickCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendJointSpeedsJoystickCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendJointSpeedsJoystickCommand-response> is deprecated: use kortex_driver-srv:SendJointSpeedsJoystickCommand-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SendJointSpeedsJoystickCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendJointSpeedsJoystickCommand-response>) ostream)
  "Serializes a message object of type '<SendJointSpeedsJoystickCommand-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendJointSpeedsJoystickCommand-response>) istream)
  "Deserializes a message object of type '<SendJointSpeedsJoystickCommand-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendJointSpeedsJoystickCommand-response>)))
  "Returns string type for a service object of type '<SendJointSpeedsJoystickCommand-response>"
  "kortex_driver/SendJointSpeedsJoystickCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendJointSpeedsJoystickCommand-response)))
  "Returns string type for a service object of type 'SendJointSpeedsJoystickCommand-response"
  "kortex_driver/SendJointSpeedsJoystickCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendJointSpeedsJoystickCommand-response>)))
  "Returns md5sum for a message object of type '<SendJointSpeedsJoystickCommand-response>"
  "35bff15135e19b4099e6a92d5e7d08d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendJointSpeedsJoystickCommand-response)))
  "Returns md5sum for a message object of type 'SendJointSpeedsJoystickCommand-response"
  "35bff15135e19b4099e6a92d5e7d08d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendJointSpeedsJoystickCommand-response>)))
  "Returns full string definition for message of type '<SendJointSpeedsJoystickCommand-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendJointSpeedsJoystickCommand-response)))
  "Returns full string definition for message of type 'SendJointSpeedsJoystickCommand-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendJointSpeedsJoystickCommand-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendJointSpeedsJoystickCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SendJointSpeedsJoystickCommand-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SendJointSpeedsJoystickCommand)))
  'SendJointSpeedsJoystickCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SendJointSpeedsJoystickCommand)))
  'SendJointSpeedsJoystickCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendJointSpeedsJoystickCommand)))
  "Returns string type for a service object of type '<SendJointSpeedsJoystickCommand>"
  "kortex_driver/SendJointSpeedsJoystickCommand")