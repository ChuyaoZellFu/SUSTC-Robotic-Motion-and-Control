; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetAllJointsSpeedHardLimitation-request.msg.html

(cl:defclass <GetAllJointsSpeedHardLimitation-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetAllJointsSpeedHardLimitation-request (<GetAllJointsSpeedHardLimitation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllJointsSpeedHardLimitation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllJointsSpeedHardLimitation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllJointsSpeedHardLimitation-request> is deprecated: use kortex_driver-srv:GetAllJointsSpeedHardLimitation-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetAllJointsSpeedHardLimitation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllJointsSpeedHardLimitation-request>) ostream)
  "Serializes a message object of type '<GetAllJointsSpeedHardLimitation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllJointsSpeedHardLimitation-request>) istream)
  "Deserializes a message object of type '<GetAllJointsSpeedHardLimitation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllJointsSpeedHardLimitation-request>)))
  "Returns string type for a service object of type '<GetAllJointsSpeedHardLimitation-request>"
  "kortex_driver/GetAllJointsSpeedHardLimitationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllJointsSpeedHardLimitation-request)))
  "Returns string type for a service object of type 'GetAllJointsSpeedHardLimitation-request"
  "kortex_driver/GetAllJointsSpeedHardLimitationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllJointsSpeedHardLimitation-request>)))
  "Returns md5sum for a message object of type '<GetAllJointsSpeedHardLimitation-request>"
  "80ab5247f79a646a096a6bb5bc451a7d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllJointsSpeedHardLimitation-request)))
  "Returns md5sum for a message object of type 'GetAllJointsSpeedHardLimitation-request"
  "80ab5247f79a646a096a6bb5bc451a7d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllJointsSpeedHardLimitation-request>)))
  "Returns full string definition for message of type '<GetAllJointsSpeedHardLimitation-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllJointsSpeedHardLimitation-request)))
  "Returns full string definition for message of type 'GetAllJointsSpeedHardLimitation-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllJointsSpeedHardLimitation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllJointsSpeedHardLimitation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllJointsSpeedHardLimitation-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetAllJointsSpeedHardLimitation-response.msg.html

(cl:defclass <GetAllJointsSpeedHardLimitation-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:JointsLimitationsList
    :initform (cl:make-instance 'kortex_driver-msg:JointsLimitationsList)))
)

(cl:defclass GetAllJointsSpeedHardLimitation-response (<GetAllJointsSpeedHardLimitation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllJointsSpeedHardLimitation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllJointsSpeedHardLimitation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllJointsSpeedHardLimitation-response> is deprecated: use kortex_driver-srv:GetAllJointsSpeedHardLimitation-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetAllJointsSpeedHardLimitation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllJointsSpeedHardLimitation-response>) ostream)
  "Serializes a message object of type '<GetAllJointsSpeedHardLimitation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllJointsSpeedHardLimitation-response>) istream)
  "Deserializes a message object of type '<GetAllJointsSpeedHardLimitation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllJointsSpeedHardLimitation-response>)))
  "Returns string type for a service object of type '<GetAllJointsSpeedHardLimitation-response>"
  "kortex_driver/GetAllJointsSpeedHardLimitationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllJointsSpeedHardLimitation-response)))
  "Returns string type for a service object of type 'GetAllJointsSpeedHardLimitation-response"
  "kortex_driver/GetAllJointsSpeedHardLimitationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllJointsSpeedHardLimitation-response>)))
  "Returns md5sum for a message object of type '<GetAllJointsSpeedHardLimitation-response>"
  "80ab5247f79a646a096a6bb5bc451a7d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllJointsSpeedHardLimitation-response)))
  "Returns md5sum for a message object of type 'GetAllJointsSpeedHardLimitation-response"
  "80ab5247f79a646a096a6bb5bc451a7d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllJointsSpeedHardLimitation-response>)))
  "Returns full string definition for message of type '<GetAllJointsSpeedHardLimitation-response>"
  (cl:format cl:nil "JointsLimitationsList output~%~%================================================================================~%MSG: kortex_driver/JointsLimitationsList~%~%JointLimitation[] joints_limitations~%================================================================================~%MSG: kortex_driver/JointLimitation~%~%uint32 joint_identifier~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllJointsSpeedHardLimitation-response)))
  "Returns full string definition for message of type 'GetAllJointsSpeedHardLimitation-response"
  (cl:format cl:nil "JointsLimitationsList output~%~%================================================================================~%MSG: kortex_driver/JointsLimitationsList~%~%JointLimitation[] joints_limitations~%================================================================================~%MSG: kortex_driver/JointLimitation~%~%uint32 joint_identifier~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllJointsSpeedHardLimitation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllJointsSpeedHardLimitation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllJointsSpeedHardLimitation-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAllJointsSpeedHardLimitation)))
  'GetAllJointsSpeedHardLimitation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAllJointsSpeedHardLimitation)))
  'GetAllJointsSpeedHardLimitation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllJointsSpeedHardLimitation)))
  "Returns string type for a service object of type '<GetAllJointsSpeedHardLimitation>"
  "kortex_driver/GetAllJointsSpeedHardLimitation")