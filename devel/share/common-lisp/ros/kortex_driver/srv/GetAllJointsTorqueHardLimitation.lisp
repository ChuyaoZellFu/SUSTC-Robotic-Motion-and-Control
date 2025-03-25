; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetAllJointsTorqueHardLimitation-request.msg.html

(cl:defclass <GetAllJointsTorqueHardLimitation-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetAllJointsTorqueHardLimitation-request (<GetAllJointsTorqueHardLimitation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllJointsTorqueHardLimitation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllJointsTorqueHardLimitation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllJointsTorqueHardLimitation-request> is deprecated: use kortex_driver-srv:GetAllJointsTorqueHardLimitation-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetAllJointsTorqueHardLimitation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllJointsTorqueHardLimitation-request>) ostream)
  "Serializes a message object of type '<GetAllJointsTorqueHardLimitation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllJointsTorqueHardLimitation-request>) istream)
  "Deserializes a message object of type '<GetAllJointsTorqueHardLimitation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllJointsTorqueHardLimitation-request>)))
  "Returns string type for a service object of type '<GetAllJointsTorqueHardLimitation-request>"
  "kortex_driver/GetAllJointsTorqueHardLimitationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllJointsTorqueHardLimitation-request)))
  "Returns string type for a service object of type 'GetAllJointsTorqueHardLimitation-request"
  "kortex_driver/GetAllJointsTorqueHardLimitationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllJointsTorqueHardLimitation-request>)))
  "Returns md5sum for a message object of type '<GetAllJointsTorqueHardLimitation-request>"
  "80ab5247f79a646a096a6bb5bc451a7d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllJointsTorqueHardLimitation-request)))
  "Returns md5sum for a message object of type 'GetAllJointsTorqueHardLimitation-request"
  "80ab5247f79a646a096a6bb5bc451a7d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllJointsTorqueHardLimitation-request>)))
  "Returns full string definition for message of type '<GetAllJointsTorqueHardLimitation-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllJointsTorqueHardLimitation-request)))
  "Returns full string definition for message of type 'GetAllJointsTorqueHardLimitation-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllJointsTorqueHardLimitation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllJointsTorqueHardLimitation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllJointsTorqueHardLimitation-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetAllJointsTorqueHardLimitation-response.msg.html

(cl:defclass <GetAllJointsTorqueHardLimitation-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:JointsLimitationsList
    :initform (cl:make-instance 'kortex_driver-msg:JointsLimitationsList)))
)

(cl:defclass GetAllJointsTorqueHardLimitation-response (<GetAllJointsTorqueHardLimitation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllJointsTorqueHardLimitation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllJointsTorqueHardLimitation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllJointsTorqueHardLimitation-response> is deprecated: use kortex_driver-srv:GetAllJointsTorqueHardLimitation-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetAllJointsTorqueHardLimitation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllJointsTorqueHardLimitation-response>) ostream)
  "Serializes a message object of type '<GetAllJointsTorqueHardLimitation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllJointsTorqueHardLimitation-response>) istream)
  "Deserializes a message object of type '<GetAllJointsTorqueHardLimitation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllJointsTorqueHardLimitation-response>)))
  "Returns string type for a service object of type '<GetAllJointsTorqueHardLimitation-response>"
  "kortex_driver/GetAllJointsTorqueHardLimitationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllJointsTorqueHardLimitation-response)))
  "Returns string type for a service object of type 'GetAllJointsTorqueHardLimitation-response"
  "kortex_driver/GetAllJointsTorqueHardLimitationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllJointsTorqueHardLimitation-response>)))
  "Returns md5sum for a message object of type '<GetAllJointsTorqueHardLimitation-response>"
  "80ab5247f79a646a096a6bb5bc451a7d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllJointsTorqueHardLimitation-response)))
  "Returns md5sum for a message object of type 'GetAllJointsTorqueHardLimitation-response"
  "80ab5247f79a646a096a6bb5bc451a7d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllJointsTorqueHardLimitation-response>)))
  "Returns full string definition for message of type '<GetAllJointsTorqueHardLimitation-response>"
  (cl:format cl:nil "JointsLimitationsList output~%~%================================================================================~%MSG: kortex_driver/JointsLimitationsList~%~%JointLimitation[] joints_limitations~%================================================================================~%MSG: kortex_driver/JointLimitation~%~%uint32 joint_identifier~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllJointsTorqueHardLimitation-response)))
  "Returns full string definition for message of type 'GetAllJointsTorqueHardLimitation-response"
  (cl:format cl:nil "JointsLimitationsList output~%~%================================================================================~%MSG: kortex_driver/JointsLimitationsList~%~%JointLimitation[] joints_limitations~%================================================================================~%MSG: kortex_driver/JointLimitation~%~%uint32 joint_identifier~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllJointsTorqueHardLimitation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllJointsTorqueHardLimitation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllJointsTorqueHardLimitation-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAllJointsTorqueHardLimitation)))
  'GetAllJointsTorqueHardLimitation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAllJointsTorqueHardLimitation)))
  'GetAllJointsTorqueHardLimitation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllJointsTorqueHardLimitation)))
  "Returns string type for a service object of type '<GetAllJointsTorqueHardLimitation>"
  "kortex_driver/GetAllJointsTorqueHardLimitation")