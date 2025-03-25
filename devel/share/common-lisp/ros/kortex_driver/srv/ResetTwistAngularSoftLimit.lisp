; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ResetTwistAngularSoftLimit-request.msg.html

(cl:defclass <ResetTwistAngularSoftLimit-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ControlConfig_ControlModeInformation
    :initform (cl:make-instance 'kortex_driver-msg:ControlConfig_ControlModeInformation)))
)

(cl:defclass ResetTwistAngularSoftLimit-request (<ResetTwistAngularSoftLimit-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetTwistAngularSoftLimit-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetTwistAngularSoftLimit-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ResetTwistAngularSoftLimit-request> is deprecated: use kortex_driver-srv:ResetTwistAngularSoftLimit-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ResetTwistAngularSoftLimit-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetTwistAngularSoftLimit-request>) ostream)
  "Serializes a message object of type '<ResetTwistAngularSoftLimit-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetTwistAngularSoftLimit-request>) istream)
  "Deserializes a message object of type '<ResetTwistAngularSoftLimit-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetTwistAngularSoftLimit-request>)))
  "Returns string type for a service object of type '<ResetTwistAngularSoftLimit-request>"
  "kortex_driver/ResetTwistAngularSoftLimitRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetTwistAngularSoftLimit-request)))
  "Returns string type for a service object of type 'ResetTwistAngularSoftLimit-request"
  "kortex_driver/ResetTwistAngularSoftLimitRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetTwistAngularSoftLimit-request>)))
  "Returns md5sum for a message object of type '<ResetTwistAngularSoftLimit-request>"
  "24d97d0cfe0dd3c18032abd06a33780b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetTwistAngularSoftLimit-request)))
  "Returns md5sum for a message object of type 'ResetTwistAngularSoftLimit-request"
  "24d97d0cfe0dd3c18032abd06a33780b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetTwistAngularSoftLimit-request>)))
  "Returns full string definition for message of type '<ResetTwistAngularSoftLimit-request>"
  (cl:format cl:nil "ControlConfig_ControlModeInformation input~%~%================================================================================~%MSG: kortex_driver/ControlConfig_ControlModeInformation~%~%uint32 control_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetTwistAngularSoftLimit-request)))
  "Returns full string definition for message of type 'ResetTwistAngularSoftLimit-request"
  (cl:format cl:nil "ControlConfig_ControlModeInformation input~%~%================================================================================~%MSG: kortex_driver/ControlConfig_ControlModeInformation~%~%uint32 control_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetTwistAngularSoftLimit-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetTwistAngularSoftLimit-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetTwistAngularSoftLimit-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ResetTwistAngularSoftLimit-response.msg.html

(cl:defclass <ResetTwistAngularSoftLimit-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:TwistAngularSoftLimit
    :initform (cl:make-instance 'kortex_driver-msg:TwistAngularSoftLimit)))
)

(cl:defclass ResetTwistAngularSoftLimit-response (<ResetTwistAngularSoftLimit-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetTwistAngularSoftLimit-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetTwistAngularSoftLimit-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ResetTwistAngularSoftLimit-response> is deprecated: use kortex_driver-srv:ResetTwistAngularSoftLimit-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ResetTwistAngularSoftLimit-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetTwistAngularSoftLimit-response>) ostream)
  "Serializes a message object of type '<ResetTwistAngularSoftLimit-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetTwistAngularSoftLimit-response>) istream)
  "Deserializes a message object of type '<ResetTwistAngularSoftLimit-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetTwistAngularSoftLimit-response>)))
  "Returns string type for a service object of type '<ResetTwistAngularSoftLimit-response>"
  "kortex_driver/ResetTwistAngularSoftLimitResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetTwistAngularSoftLimit-response)))
  "Returns string type for a service object of type 'ResetTwistAngularSoftLimit-response"
  "kortex_driver/ResetTwistAngularSoftLimitResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetTwistAngularSoftLimit-response>)))
  "Returns md5sum for a message object of type '<ResetTwistAngularSoftLimit-response>"
  "24d97d0cfe0dd3c18032abd06a33780b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetTwistAngularSoftLimit-response)))
  "Returns md5sum for a message object of type 'ResetTwistAngularSoftLimit-response"
  "24d97d0cfe0dd3c18032abd06a33780b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetTwistAngularSoftLimit-response>)))
  "Returns full string definition for message of type '<ResetTwistAngularSoftLimit-response>"
  (cl:format cl:nil "TwistAngularSoftLimit output~%~%================================================================================~%MSG: kortex_driver/TwistAngularSoftLimit~%~%uint32 control_mode~%float32 twist_angular_soft_limit~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetTwistAngularSoftLimit-response)))
  "Returns full string definition for message of type 'ResetTwistAngularSoftLimit-response"
  (cl:format cl:nil "TwistAngularSoftLimit output~%~%================================================================================~%MSG: kortex_driver/TwistAngularSoftLimit~%~%uint32 control_mode~%float32 twist_angular_soft_limit~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetTwistAngularSoftLimit-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetTwistAngularSoftLimit-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetTwistAngularSoftLimit-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetTwistAngularSoftLimit)))
  'ResetTwistAngularSoftLimit-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetTwistAngularSoftLimit)))
  'ResetTwistAngularSoftLimit-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetTwistAngularSoftLimit)))
  "Returns string type for a service object of type '<ResetTwistAngularSoftLimit>"
  "kortex_driver/ResetTwistAngularSoftLimit")