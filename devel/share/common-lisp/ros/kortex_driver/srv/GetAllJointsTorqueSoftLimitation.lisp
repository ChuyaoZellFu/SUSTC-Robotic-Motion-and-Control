; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetAllJointsTorqueSoftLimitation-request.msg.html

(cl:defclass <GetAllJointsTorqueSoftLimitation-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetAllJointsTorqueSoftLimitation-request (<GetAllJointsTorqueSoftLimitation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllJointsTorqueSoftLimitation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllJointsTorqueSoftLimitation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllJointsTorqueSoftLimitation-request> is deprecated: use kortex_driver-srv:GetAllJointsTorqueSoftLimitation-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetAllJointsTorqueSoftLimitation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllJointsTorqueSoftLimitation-request>) ostream)
  "Serializes a message object of type '<GetAllJointsTorqueSoftLimitation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllJointsTorqueSoftLimitation-request>) istream)
  "Deserializes a message object of type '<GetAllJointsTorqueSoftLimitation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllJointsTorqueSoftLimitation-request>)))
  "Returns string type for a service object of type '<GetAllJointsTorqueSoftLimitation-request>"
  "kortex_driver/GetAllJointsTorqueSoftLimitationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllJointsTorqueSoftLimitation-request)))
  "Returns string type for a service object of type 'GetAllJointsTorqueSoftLimitation-request"
  "kortex_driver/GetAllJointsTorqueSoftLimitationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllJointsTorqueSoftLimitation-request>)))
  "Returns md5sum for a message object of type '<GetAllJointsTorqueSoftLimitation-request>"
  "80ab5247f79a646a096a6bb5bc451a7d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllJointsTorqueSoftLimitation-request)))
  "Returns md5sum for a message object of type 'GetAllJointsTorqueSoftLimitation-request"
  "80ab5247f79a646a096a6bb5bc451a7d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllJointsTorqueSoftLimitation-request>)))
  "Returns full string definition for message of type '<GetAllJointsTorqueSoftLimitation-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllJointsTorqueSoftLimitation-request)))
  "Returns full string definition for message of type 'GetAllJointsTorqueSoftLimitation-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllJointsTorqueSoftLimitation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllJointsTorqueSoftLimitation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllJointsTorqueSoftLimitation-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetAllJointsTorqueSoftLimitation-response.msg.html

(cl:defclass <GetAllJointsTorqueSoftLimitation-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:JointsLimitationsList
    :initform (cl:make-instance 'kortex_driver-msg:JointsLimitationsList)))
)

(cl:defclass GetAllJointsTorqueSoftLimitation-response (<GetAllJointsTorqueSoftLimitation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllJointsTorqueSoftLimitation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllJointsTorqueSoftLimitation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllJointsTorqueSoftLimitation-response> is deprecated: use kortex_driver-srv:GetAllJointsTorqueSoftLimitation-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetAllJointsTorqueSoftLimitation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllJointsTorqueSoftLimitation-response>) ostream)
  "Serializes a message object of type '<GetAllJointsTorqueSoftLimitation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllJointsTorqueSoftLimitation-response>) istream)
  "Deserializes a message object of type '<GetAllJointsTorqueSoftLimitation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllJointsTorqueSoftLimitation-response>)))
  "Returns string type for a service object of type '<GetAllJointsTorqueSoftLimitation-response>"
  "kortex_driver/GetAllJointsTorqueSoftLimitationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllJointsTorqueSoftLimitation-response)))
  "Returns string type for a service object of type 'GetAllJointsTorqueSoftLimitation-response"
  "kortex_driver/GetAllJointsTorqueSoftLimitationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllJointsTorqueSoftLimitation-response>)))
  "Returns md5sum for a message object of type '<GetAllJointsTorqueSoftLimitation-response>"
  "80ab5247f79a646a096a6bb5bc451a7d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllJointsTorqueSoftLimitation-response)))
  "Returns md5sum for a message object of type 'GetAllJointsTorqueSoftLimitation-response"
  "80ab5247f79a646a096a6bb5bc451a7d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllJointsTorqueSoftLimitation-response>)))
  "Returns full string definition for message of type '<GetAllJointsTorqueSoftLimitation-response>"
  (cl:format cl:nil "JointsLimitationsList output~%~%================================================================================~%MSG: kortex_driver/JointsLimitationsList~%~%JointLimitation[] joints_limitations~%================================================================================~%MSG: kortex_driver/JointLimitation~%~%uint32 joint_identifier~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllJointsTorqueSoftLimitation-response)))
  "Returns full string definition for message of type 'GetAllJointsTorqueSoftLimitation-response"
  (cl:format cl:nil "JointsLimitationsList output~%~%================================================================================~%MSG: kortex_driver/JointsLimitationsList~%~%JointLimitation[] joints_limitations~%================================================================================~%MSG: kortex_driver/JointLimitation~%~%uint32 joint_identifier~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllJointsTorqueSoftLimitation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllJointsTorqueSoftLimitation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllJointsTorqueSoftLimitation-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAllJointsTorqueSoftLimitation)))
  'GetAllJointsTorqueSoftLimitation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAllJointsTorqueSoftLimitation)))
  'GetAllJointsTorqueSoftLimitation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllJointsTorqueSoftLimitation)))
  "Returns string type for a service object of type '<GetAllJointsTorqueSoftLimitation>"
  "kortex_driver/GetAllJointsTorqueSoftLimitation")