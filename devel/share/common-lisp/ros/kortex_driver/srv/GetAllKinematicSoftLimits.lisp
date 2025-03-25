; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetAllKinematicSoftLimits-request.msg.html

(cl:defclass <GetAllKinematicSoftLimits-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetAllKinematicSoftLimits-request (<GetAllKinematicSoftLimits-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllKinematicSoftLimits-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllKinematicSoftLimits-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllKinematicSoftLimits-request> is deprecated: use kortex_driver-srv:GetAllKinematicSoftLimits-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetAllKinematicSoftLimits-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllKinematicSoftLimits-request>) ostream)
  "Serializes a message object of type '<GetAllKinematicSoftLimits-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllKinematicSoftLimits-request>) istream)
  "Deserializes a message object of type '<GetAllKinematicSoftLimits-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllKinematicSoftLimits-request>)))
  "Returns string type for a service object of type '<GetAllKinematicSoftLimits-request>"
  "kortex_driver/GetAllKinematicSoftLimitsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllKinematicSoftLimits-request)))
  "Returns string type for a service object of type 'GetAllKinematicSoftLimits-request"
  "kortex_driver/GetAllKinematicSoftLimitsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllKinematicSoftLimits-request>)))
  "Returns md5sum for a message object of type '<GetAllKinematicSoftLimits-request>"
  "82a8454ad50f9bddae049fa61b8b3c3d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllKinematicSoftLimits-request)))
  "Returns md5sum for a message object of type 'GetAllKinematicSoftLimits-request"
  "82a8454ad50f9bddae049fa61b8b3c3d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllKinematicSoftLimits-request>)))
  "Returns full string definition for message of type '<GetAllKinematicSoftLimits-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllKinematicSoftLimits-request)))
  "Returns full string definition for message of type 'GetAllKinematicSoftLimits-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllKinematicSoftLimits-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllKinematicSoftLimits-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllKinematicSoftLimits-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetAllKinematicSoftLimits-response.msg.html

(cl:defclass <GetAllKinematicSoftLimits-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:KinematicLimitsList
    :initform (cl:make-instance 'kortex_driver-msg:KinematicLimitsList)))
)

(cl:defclass GetAllKinematicSoftLimits-response (<GetAllKinematicSoftLimits-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllKinematicSoftLimits-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllKinematicSoftLimits-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllKinematicSoftLimits-response> is deprecated: use kortex_driver-srv:GetAllKinematicSoftLimits-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetAllKinematicSoftLimits-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllKinematicSoftLimits-response>) ostream)
  "Serializes a message object of type '<GetAllKinematicSoftLimits-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllKinematicSoftLimits-response>) istream)
  "Deserializes a message object of type '<GetAllKinematicSoftLimits-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllKinematicSoftLimits-response>)))
  "Returns string type for a service object of type '<GetAllKinematicSoftLimits-response>"
  "kortex_driver/GetAllKinematicSoftLimitsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllKinematicSoftLimits-response)))
  "Returns string type for a service object of type 'GetAllKinematicSoftLimits-response"
  "kortex_driver/GetAllKinematicSoftLimitsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllKinematicSoftLimits-response>)))
  "Returns md5sum for a message object of type '<GetAllKinematicSoftLimits-response>"
  "82a8454ad50f9bddae049fa61b8b3c3d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllKinematicSoftLimits-response)))
  "Returns md5sum for a message object of type 'GetAllKinematicSoftLimits-response"
  "82a8454ad50f9bddae049fa61b8b3c3d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllKinematicSoftLimits-response>)))
  "Returns full string definition for message of type '<GetAllKinematicSoftLimits-response>"
  (cl:format cl:nil "KinematicLimitsList output~%~%================================================================================~%MSG: kortex_driver/KinematicLimitsList~%~%KinematicLimits[] kinematic_limits_list~%================================================================================~%MSG: kortex_driver/KinematicLimits~%~%uint32 control_mode~%float32 twist_linear~%float32 twist_angular~%float32[] joint_speed_limits~%float32[] joint_acceleration_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllKinematicSoftLimits-response)))
  "Returns full string definition for message of type 'GetAllKinematicSoftLimits-response"
  (cl:format cl:nil "KinematicLimitsList output~%~%================================================================================~%MSG: kortex_driver/KinematicLimitsList~%~%KinematicLimits[] kinematic_limits_list~%================================================================================~%MSG: kortex_driver/KinematicLimits~%~%uint32 control_mode~%float32 twist_linear~%float32 twist_angular~%float32[] joint_speed_limits~%float32[] joint_acceleration_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllKinematicSoftLimits-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllKinematicSoftLimits-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllKinematicSoftLimits-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAllKinematicSoftLimits)))
  'GetAllKinematicSoftLimits-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAllKinematicSoftLimits)))
  'GetAllKinematicSoftLimits-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllKinematicSoftLimits)))
  "Returns string type for a service object of type '<GetAllKinematicSoftLimits>"
  "kortex_driver/GetAllKinematicSoftLimits")