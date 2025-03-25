; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetAllJointsSpeedSoftLimitation-request.msg.html

(cl:defclass <GetAllJointsSpeedSoftLimitation-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetAllJointsSpeedSoftLimitation-request (<GetAllJointsSpeedSoftLimitation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllJointsSpeedSoftLimitation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllJointsSpeedSoftLimitation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllJointsSpeedSoftLimitation-request> is deprecated: use kortex_driver-srv:GetAllJointsSpeedSoftLimitation-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetAllJointsSpeedSoftLimitation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllJointsSpeedSoftLimitation-request>) ostream)
  "Serializes a message object of type '<GetAllJointsSpeedSoftLimitation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllJointsSpeedSoftLimitation-request>) istream)
  "Deserializes a message object of type '<GetAllJointsSpeedSoftLimitation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllJointsSpeedSoftLimitation-request>)))
  "Returns string type for a service object of type '<GetAllJointsSpeedSoftLimitation-request>"
  "kortex_driver/GetAllJointsSpeedSoftLimitationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllJointsSpeedSoftLimitation-request)))
  "Returns string type for a service object of type 'GetAllJointsSpeedSoftLimitation-request"
  "kortex_driver/GetAllJointsSpeedSoftLimitationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllJointsSpeedSoftLimitation-request>)))
  "Returns md5sum for a message object of type '<GetAllJointsSpeedSoftLimitation-request>"
  "80ab5247f79a646a096a6bb5bc451a7d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllJointsSpeedSoftLimitation-request)))
  "Returns md5sum for a message object of type 'GetAllJointsSpeedSoftLimitation-request"
  "80ab5247f79a646a096a6bb5bc451a7d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllJointsSpeedSoftLimitation-request>)))
  "Returns full string definition for message of type '<GetAllJointsSpeedSoftLimitation-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllJointsSpeedSoftLimitation-request)))
  "Returns full string definition for message of type 'GetAllJointsSpeedSoftLimitation-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllJointsSpeedSoftLimitation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllJointsSpeedSoftLimitation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllJointsSpeedSoftLimitation-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetAllJointsSpeedSoftLimitation-response.msg.html

(cl:defclass <GetAllJointsSpeedSoftLimitation-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:JointsLimitationsList
    :initform (cl:make-instance 'kortex_driver-msg:JointsLimitationsList)))
)

(cl:defclass GetAllJointsSpeedSoftLimitation-response (<GetAllJointsSpeedSoftLimitation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllJointsSpeedSoftLimitation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllJointsSpeedSoftLimitation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllJointsSpeedSoftLimitation-response> is deprecated: use kortex_driver-srv:GetAllJointsSpeedSoftLimitation-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetAllJointsSpeedSoftLimitation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllJointsSpeedSoftLimitation-response>) ostream)
  "Serializes a message object of type '<GetAllJointsSpeedSoftLimitation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllJointsSpeedSoftLimitation-response>) istream)
  "Deserializes a message object of type '<GetAllJointsSpeedSoftLimitation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllJointsSpeedSoftLimitation-response>)))
  "Returns string type for a service object of type '<GetAllJointsSpeedSoftLimitation-response>"
  "kortex_driver/GetAllJointsSpeedSoftLimitationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllJointsSpeedSoftLimitation-response)))
  "Returns string type for a service object of type 'GetAllJointsSpeedSoftLimitation-response"
  "kortex_driver/GetAllJointsSpeedSoftLimitationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllJointsSpeedSoftLimitation-response>)))
  "Returns md5sum for a message object of type '<GetAllJointsSpeedSoftLimitation-response>"
  "80ab5247f79a646a096a6bb5bc451a7d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllJointsSpeedSoftLimitation-response)))
  "Returns md5sum for a message object of type 'GetAllJointsSpeedSoftLimitation-response"
  "80ab5247f79a646a096a6bb5bc451a7d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllJointsSpeedSoftLimitation-response>)))
  "Returns full string definition for message of type '<GetAllJointsSpeedSoftLimitation-response>"
  (cl:format cl:nil "JointsLimitationsList output~%~%================================================================================~%MSG: kortex_driver/JointsLimitationsList~%~%JointLimitation[] joints_limitations~%================================================================================~%MSG: kortex_driver/JointLimitation~%~%uint32 joint_identifier~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllJointsSpeedSoftLimitation-response)))
  "Returns full string definition for message of type 'GetAllJointsSpeedSoftLimitation-response"
  (cl:format cl:nil "JointsLimitationsList output~%~%================================================================================~%MSG: kortex_driver/JointsLimitationsList~%~%JointLimitation[] joints_limitations~%================================================================================~%MSG: kortex_driver/JointLimitation~%~%uint32 joint_identifier~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllJointsSpeedSoftLimitation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllJointsSpeedSoftLimitation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllJointsSpeedSoftLimitation-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAllJointsSpeedSoftLimitation)))
  'GetAllJointsSpeedSoftLimitation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAllJointsSpeedSoftLimitation)))
  'GetAllJointsSpeedSoftLimitation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllJointsSpeedSoftLimitation)))
  "Returns string type for a service object of type '<GetAllJointsSpeedSoftLimitation>"
  "kortex_driver/GetAllJointsSpeedSoftLimitation")