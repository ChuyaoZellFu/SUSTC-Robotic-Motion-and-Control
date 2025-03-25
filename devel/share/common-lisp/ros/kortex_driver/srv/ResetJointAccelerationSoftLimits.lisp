; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ResetJointAccelerationSoftLimits-request.msg.html

(cl:defclass <ResetJointAccelerationSoftLimits-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ControlConfig_ControlModeInformation
    :initform (cl:make-instance 'kortex_driver-msg:ControlConfig_ControlModeInformation)))
)

(cl:defclass ResetJointAccelerationSoftLimits-request (<ResetJointAccelerationSoftLimits-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetJointAccelerationSoftLimits-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetJointAccelerationSoftLimits-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ResetJointAccelerationSoftLimits-request> is deprecated: use kortex_driver-srv:ResetJointAccelerationSoftLimits-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ResetJointAccelerationSoftLimits-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetJointAccelerationSoftLimits-request>) ostream)
  "Serializes a message object of type '<ResetJointAccelerationSoftLimits-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetJointAccelerationSoftLimits-request>) istream)
  "Deserializes a message object of type '<ResetJointAccelerationSoftLimits-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetJointAccelerationSoftLimits-request>)))
  "Returns string type for a service object of type '<ResetJointAccelerationSoftLimits-request>"
  "kortex_driver/ResetJointAccelerationSoftLimitsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetJointAccelerationSoftLimits-request)))
  "Returns string type for a service object of type 'ResetJointAccelerationSoftLimits-request"
  "kortex_driver/ResetJointAccelerationSoftLimitsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetJointAccelerationSoftLimits-request>)))
  "Returns md5sum for a message object of type '<ResetJointAccelerationSoftLimits-request>"
  "6dba409e37ab85922f7777c936234d59")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetJointAccelerationSoftLimits-request)))
  "Returns md5sum for a message object of type 'ResetJointAccelerationSoftLimits-request"
  "6dba409e37ab85922f7777c936234d59")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetJointAccelerationSoftLimits-request>)))
  "Returns full string definition for message of type '<ResetJointAccelerationSoftLimits-request>"
  (cl:format cl:nil "ControlConfig_ControlModeInformation input~%~%================================================================================~%MSG: kortex_driver/ControlConfig_ControlModeInformation~%~%uint32 control_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetJointAccelerationSoftLimits-request)))
  "Returns full string definition for message of type 'ResetJointAccelerationSoftLimits-request"
  (cl:format cl:nil "ControlConfig_ControlModeInformation input~%~%================================================================================~%MSG: kortex_driver/ControlConfig_ControlModeInformation~%~%uint32 control_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetJointAccelerationSoftLimits-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetJointAccelerationSoftLimits-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetJointAccelerationSoftLimits-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ResetJointAccelerationSoftLimits-response.msg.html

(cl:defclass <ResetJointAccelerationSoftLimits-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:JointAccelerationSoftLimits
    :initform (cl:make-instance 'kortex_driver-msg:JointAccelerationSoftLimits)))
)

(cl:defclass ResetJointAccelerationSoftLimits-response (<ResetJointAccelerationSoftLimits-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetJointAccelerationSoftLimits-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetJointAccelerationSoftLimits-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ResetJointAccelerationSoftLimits-response> is deprecated: use kortex_driver-srv:ResetJointAccelerationSoftLimits-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ResetJointAccelerationSoftLimits-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetJointAccelerationSoftLimits-response>) ostream)
  "Serializes a message object of type '<ResetJointAccelerationSoftLimits-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetJointAccelerationSoftLimits-response>) istream)
  "Deserializes a message object of type '<ResetJointAccelerationSoftLimits-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetJointAccelerationSoftLimits-response>)))
  "Returns string type for a service object of type '<ResetJointAccelerationSoftLimits-response>"
  "kortex_driver/ResetJointAccelerationSoftLimitsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetJointAccelerationSoftLimits-response)))
  "Returns string type for a service object of type 'ResetJointAccelerationSoftLimits-response"
  "kortex_driver/ResetJointAccelerationSoftLimitsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetJointAccelerationSoftLimits-response>)))
  "Returns md5sum for a message object of type '<ResetJointAccelerationSoftLimits-response>"
  "6dba409e37ab85922f7777c936234d59")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetJointAccelerationSoftLimits-response)))
  "Returns md5sum for a message object of type 'ResetJointAccelerationSoftLimits-response"
  "6dba409e37ab85922f7777c936234d59")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetJointAccelerationSoftLimits-response>)))
  "Returns full string definition for message of type '<ResetJointAccelerationSoftLimits-response>"
  (cl:format cl:nil "JointAccelerationSoftLimits output~%~%================================================================================~%MSG: kortex_driver/JointAccelerationSoftLimits~%~%uint32 control_mode~%float32[] joint_acceleration_soft_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetJointAccelerationSoftLimits-response)))
  "Returns full string definition for message of type 'ResetJointAccelerationSoftLimits-response"
  (cl:format cl:nil "JointAccelerationSoftLimits output~%~%================================================================================~%MSG: kortex_driver/JointAccelerationSoftLimits~%~%uint32 control_mode~%float32[] joint_acceleration_soft_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetJointAccelerationSoftLimits-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetJointAccelerationSoftLimits-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetJointAccelerationSoftLimits-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetJointAccelerationSoftLimits)))
  'ResetJointAccelerationSoftLimits-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetJointAccelerationSoftLimits)))
  'ResetJointAccelerationSoftLimits-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetJointAccelerationSoftLimits)))
  "Returns string type for a service object of type '<ResetJointAccelerationSoftLimits>"
  "kortex_driver/ResetJointAccelerationSoftLimits")