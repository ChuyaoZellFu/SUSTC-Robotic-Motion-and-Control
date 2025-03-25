; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetKinematicSoftLimits-request.msg.html

(cl:defclass <GetKinematicSoftLimits-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ControlConfig_ControlModeInformation
    :initform (cl:make-instance 'kortex_driver-msg:ControlConfig_ControlModeInformation)))
)

(cl:defclass GetKinematicSoftLimits-request (<GetKinematicSoftLimits-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetKinematicSoftLimits-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetKinematicSoftLimits-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetKinematicSoftLimits-request> is deprecated: use kortex_driver-srv:GetKinematicSoftLimits-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetKinematicSoftLimits-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetKinematicSoftLimits-request>) ostream)
  "Serializes a message object of type '<GetKinematicSoftLimits-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetKinematicSoftLimits-request>) istream)
  "Deserializes a message object of type '<GetKinematicSoftLimits-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetKinematicSoftLimits-request>)))
  "Returns string type for a service object of type '<GetKinematicSoftLimits-request>"
  "kortex_driver/GetKinematicSoftLimitsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetKinematicSoftLimits-request)))
  "Returns string type for a service object of type 'GetKinematicSoftLimits-request"
  "kortex_driver/GetKinematicSoftLimitsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetKinematicSoftLimits-request>)))
  "Returns md5sum for a message object of type '<GetKinematicSoftLimits-request>"
  "8372b01bbf6aa0f718ce44248decf90a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetKinematicSoftLimits-request)))
  "Returns md5sum for a message object of type 'GetKinematicSoftLimits-request"
  "8372b01bbf6aa0f718ce44248decf90a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetKinematicSoftLimits-request>)))
  "Returns full string definition for message of type '<GetKinematicSoftLimits-request>"
  (cl:format cl:nil "ControlConfig_ControlModeInformation input~%~%================================================================================~%MSG: kortex_driver/ControlConfig_ControlModeInformation~%~%uint32 control_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetKinematicSoftLimits-request)))
  "Returns full string definition for message of type 'GetKinematicSoftLimits-request"
  (cl:format cl:nil "ControlConfig_ControlModeInformation input~%~%================================================================================~%MSG: kortex_driver/ControlConfig_ControlModeInformation~%~%uint32 control_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetKinematicSoftLimits-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetKinematicSoftLimits-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetKinematicSoftLimits-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetKinematicSoftLimits-response.msg.html

(cl:defclass <GetKinematicSoftLimits-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:KinematicLimits
    :initform (cl:make-instance 'kortex_driver-msg:KinematicLimits)))
)

(cl:defclass GetKinematicSoftLimits-response (<GetKinematicSoftLimits-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetKinematicSoftLimits-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetKinematicSoftLimits-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetKinematicSoftLimits-response> is deprecated: use kortex_driver-srv:GetKinematicSoftLimits-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetKinematicSoftLimits-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetKinematicSoftLimits-response>) ostream)
  "Serializes a message object of type '<GetKinematicSoftLimits-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetKinematicSoftLimits-response>) istream)
  "Deserializes a message object of type '<GetKinematicSoftLimits-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetKinematicSoftLimits-response>)))
  "Returns string type for a service object of type '<GetKinematicSoftLimits-response>"
  "kortex_driver/GetKinematicSoftLimitsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetKinematicSoftLimits-response)))
  "Returns string type for a service object of type 'GetKinematicSoftLimits-response"
  "kortex_driver/GetKinematicSoftLimitsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetKinematicSoftLimits-response>)))
  "Returns md5sum for a message object of type '<GetKinematicSoftLimits-response>"
  "8372b01bbf6aa0f718ce44248decf90a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetKinematicSoftLimits-response)))
  "Returns md5sum for a message object of type 'GetKinematicSoftLimits-response"
  "8372b01bbf6aa0f718ce44248decf90a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetKinematicSoftLimits-response>)))
  "Returns full string definition for message of type '<GetKinematicSoftLimits-response>"
  (cl:format cl:nil "KinematicLimits output~%~%================================================================================~%MSG: kortex_driver/KinematicLimits~%~%uint32 control_mode~%float32 twist_linear~%float32 twist_angular~%float32[] joint_speed_limits~%float32[] joint_acceleration_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetKinematicSoftLimits-response)))
  "Returns full string definition for message of type 'GetKinematicSoftLimits-response"
  (cl:format cl:nil "KinematicLimits output~%~%================================================================================~%MSG: kortex_driver/KinematicLimits~%~%uint32 control_mode~%float32 twist_linear~%float32 twist_angular~%float32[] joint_speed_limits~%float32[] joint_acceleration_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetKinematicSoftLimits-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetKinematicSoftLimits-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetKinematicSoftLimits-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetKinematicSoftLimits)))
  'GetKinematicSoftLimits-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetKinematicSoftLimits)))
  'GetKinematicSoftLimits-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetKinematicSoftLimits)))
  "Returns string type for a service object of type '<GetKinematicSoftLimits>"
  "kortex_driver/GetKinematicSoftLimits")