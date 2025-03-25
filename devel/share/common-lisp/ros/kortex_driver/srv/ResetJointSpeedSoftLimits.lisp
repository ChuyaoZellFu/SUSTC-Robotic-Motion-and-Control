; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ResetJointSpeedSoftLimits-request.msg.html

(cl:defclass <ResetJointSpeedSoftLimits-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ControlConfig_ControlModeInformation
    :initform (cl:make-instance 'kortex_driver-msg:ControlConfig_ControlModeInformation)))
)

(cl:defclass ResetJointSpeedSoftLimits-request (<ResetJointSpeedSoftLimits-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetJointSpeedSoftLimits-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetJointSpeedSoftLimits-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ResetJointSpeedSoftLimits-request> is deprecated: use kortex_driver-srv:ResetJointSpeedSoftLimits-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ResetJointSpeedSoftLimits-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetJointSpeedSoftLimits-request>) ostream)
  "Serializes a message object of type '<ResetJointSpeedSoftLimits-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetJointSpeedSoftLimits-request>) istream)
  "Deserializes a message object of type '<ResetJointSpeedSoftLimits-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetJointSpeedSoftLimits-request>)))
  "Returns string type for a service object of type '<ResetJointSpeedSoftLimits-request>"
  "kortex_driver/ResetJointSpeedSoftLimitsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetJointSpeedSoftLimits-request)))
  "Returns string type for a service object of type 'ResetJointSpeedSoftLimits-request"
  "kortex_driver/ResetJointSpeedSoftLimitsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetJointSpeedSoftLimits-request>)))
  "Returns md5sum for a message object of type '<ResetJointSpeedSoftLimits-request>"
  "7d22c334332447e55a639a3764ac0a17")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetJointSpeedSoftLimits-request)))
  "Returns md5sum for a message object of type 'ResetJointSpeedSoftLimits-request"
  "7d22c334332447e55a639a3764ac0a17")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetJointSpeedSoftLimits-request>)))
  "Returns full string definition for message of type '<ResetJointSpeedSoftLimits-request>"
  (cl:format cl:nil "ControlConfig_ControlModeInformation input~%~%================================================================================~%MSG: kortex_driver/ControlConfig_ControlModeInformation~%~%uint32 control_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetJointSpeedSoftLimits-request)))
  "Returns full string definition for message of type 'ResetJointSpeedSoftLimits-request"
  (cl:format cl:nil "ControlConfig_ControlModeInformation input~%~%================================================================================~%MSG: kortex_driver/ControlConfig_ControlModeInformation~%~%uint32 control_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetJointSpeedSoftLimits-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetJointSpeedSoftLimits-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetJointSpeedSoftLimits-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ResetJointSpeedSoftLimits-response.msg.html

(cl:defclass <ResetJointSpeedSoftLimits-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:JointSpeedSoftLimits
    :initform (cl:make-instance 'kortex_driver-msg:JointSpeedSoftLimits)))
)

(cl:defclass ResetJointSpeedSoftLimits-response (<ResetJointSpeedSoftLimits-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetJointSpeedSoftLimits-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetJointSpeedSoftLimits-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ResetJointSpeedSoftLimits-response> is deprecated: use kortex_driver-srv:ResetJointSpeedSoftLimits-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ResetJointSpeedSoftLimits-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetJointSpeedSoftLimits-response>) ostream)
  "Serializes a message object of type '<ResetJointSpeedSoftLimits-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetJointSpeedSoftLimits-response>) istream)
  "Deserializes a message object of type '<ResetJointSpeedSoftLimits-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetJointSpeedSoftLimits-response>)))
  "Returns string type for a service object of type '<ResetJointSpeedSoftLimits-response>"
  "kortex_driver/ResetJointSpeedSoftLimitsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetJointSpeedSoftLimits-response)))
  "Returns string type for a service object of type 'ResetJointSpeedSoftLimits-response"
  "kortex_driver/ResetJointSpeedSoftLimitsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetJointSpeedSoftLimits-response>)))
  "Returns md5sum for a message object of type '<ResetJointSpeedSoftLimits-response>"
  "7d22c334332447e55a639a3764ac0a17")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetJointSpeedSoftLimits-response)))
  "Returns md5sum for a message object of type 'ResetJointSpeedSoftLimits-response"
  "7d22c334332447e55a639a3764ac0a17")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetJointSpeedSoftLimits-response>)))
  "Returns full string definition for message of type '<ResetJointSpeedSoftLimits-response>"
  (cl:format cl:nil "JointSpeedSoftLimits output~%~%================================================================================~%MSG: kortex_driver/JointSpeedSoftLimits~%~%uint32 control_mode~%float32[] joint_speed_soft_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetJointSpeedSoftLimits-response)))
  "Returns full string definition for message of type 'ResetJointSpeedSoftLimits-response"
  (cl:format cl:nil "JointSpeedSoftLimits output~%~%================================================================================~%MSG: kortex_driver/JointSpeedSoftLimits~%~%uint32 control_mode~%float32[] joint_speed_soft_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetJointSpeedSoftLimits-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetJointSpeedSoftLimits-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetJointSpeedSoftLimits-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetJointSpeedSoftLimits)))
  'ResetJointSpeedSoftLimits-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetJointSpeedSoftLimits)))
  'ResetJointSpeedSoftLimits-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetJointSpeedSoftLimits)))
  "Returns string type for a service object of type '<ResetJointSpeedSoftLimits>"
  "kortex_driver/ResetJointSpeedSoftLimits")