; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ResetTwistLinearSoftLimit-request.msg.html

(cl:defclass <ResetTwistLinearSoftLimit-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ControlConfig_ControlModeInformation
    :initform (cl:make-instance 'kortex_driver-msg:ControlConfig_ControlModeInformation)))
)

(cl:defclass ResetTwistLinearSoftLimit-request (<ResetTwistLinearSoftLimit-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetTwistLinearSoftLimit-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetTwistLinearSoftLimit-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ResetTwistLinearSoftLimit-request> is deprecated: use kortex_driver-srv:ResetTwistLinearSoftLimit-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ResetTwistLinearSoftLimit-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetTwistLinearSoftLimit-request>) ostream)
  "Serializes a message object of type '<ResetTwistLinearSoftLimit-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetTwistLinearSoftLimit-request>) istream)
  "Deserializes a message object of type '<ResetTwistLinearSoftLimit-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetTwistLinearSoftLimit-request>)))
  "Returns string type for a service object of type '<ResetTwistLinearSoftLimit-request>"
  "kortex_driver/ResetTwistLinearSoftLimitRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetTwistLinearSoftLimit-request)))
  "Returns string type for a service object of type 'ResetTwistLinearSoftLimit-request"
  "kortex_driver/ResetTwistLinearSoftLimitRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetTwistLinearSoftLimit-request>)))
  "Returns md5sum for a message object of type '<ResetTwistLinearSoftLimit-request>"
  "277326cfb7b831ac01de5d007b623790")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetTwistLinearSoftLimit-request)))
  "Returns md5sum for a message object of type 'ResetTwistLinearSoftLimit-request"
  "277326cfb7b831ac01de5d007b623790")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetTwistLinearSoftLimit-request>)))
  "Returns full string definition for message of type '<ResetTwistLinearSoftLimit-request>"
  (cl:format cl:nil "ControlConfig_ControlModeInformation input~%~%================================================================================~%MSG: kortex_driver/ControlConfig_ControlModeInformation~%~%uint32 control_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetTwistLinearSoftLimit-request)))
  "Returns full string definition for message of type 'ResetTwistLinearSoftLimit-request"
  (cl:format cl:nil "ControlConfig_ControlModeInformation input~%~%================================================================================~%MSG: kortex_driver/ControlConfig_ControlModeInformation~%~%uint32 control_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetTwistLinearSoftLimit-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetTwistLinearSoftLimit-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetTwistLinearSoftLimit-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ResetTwistLinearSoftLimit-response.msg.html

(cl:defclass <ResetTwistLinearSoftLimit-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:TwistLinearSoftLimit
    :initform (cl:make-instance 'kortex_driver-msg:TwistLinearSoftLimit)))
)

(cl:defclass ResetTwistLinearSoftLimit-response (<ResetTwistLinearSoftLimit-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetTwistLinearSoftLimit-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetTwistLinearSoftLimit-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ResetTwistLinearSoftLimit-response> is deprecated: use kortex_driver-srv:ResetTwistLinearSoftLimit-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ResetTwistLinearSoftLimit-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetTwistLinearSoftLimit-response>) ostream)
  "Serializes a message object of type '<ResetTwistLinearSoftLimit-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetTwistLinearSoftLimit-response>) istream)
  "Deserializes a message object of type '<ResetTwistLinearSoftLimit-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetTwistLinearSoftLimit-response>)))
  "Returns string type for a service object of type '<ResetTwistLinearSoftLimit-response>"
  "kortex_driver/ResetTwistLinearSoftLimitResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetTwistLinearSoftLimit-response)))
  "Returns string type for a service object of type 'ResetTwistLinearSoftLimit-response"
  "kortex_driver/ResetTwistLinearSoftLimitResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetTwistLinearSoftLimit-response>)))
  "Returns md5sum for a message object of type '<ResetTwistLinearSoftLimit-response>"
  "277326cfb7b831ac01de5d007b623790")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetTwistLinearSoftLimit-response)))
  "Returns md5sum for a message object of type 'ResetTwistLinearSoftLimit-response"
  "277326cfb7b831ac01de5d007b623790")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetTwistLinearSoftLimit-response>)))
  "Returns full string definition for message of type '<ResetTwistLinearSoftLimit-response>"
  (cl:format cl:nil "TwistLinearSoftLimit output~%~%================================================================================~%MSG: kortex_driver/TwistLinearSoftLimit~%~%uint32 control_mode~%float32 twist_linear_soft_limit~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetTwistLinearSoftLimit-response)))
  "Returns full string definition for message of type 'ResetTwistLinearSoftLimit-response"
  (cl:format cl:nil "TwistLinearSoftLimit output~%~%================================================================================~%MSG: kortex_driver/TwistLinearSoftLimit~%~%uint32 control_mode~%float32 twist_linear_soft_limit~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetTwistLinearSoftLimit-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetTwistLinearSoftLimit-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetTwistLinearSoftLimit-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetTwistLinearSoftLimit)))
  'ResetTwistLinearSoftLimit-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetTwistLinearSoftLimit)))
  'ResetTwistLinearSoftLimit-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetTwistLinearSoftLimit)))
  "Returns string type for a service object of type '<ResetTwistLinearSoftLimit>"
  "kortex_driver/ResetTwistLinearSoftLimit")