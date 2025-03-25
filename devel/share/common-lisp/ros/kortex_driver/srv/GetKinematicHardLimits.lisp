; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetKinematicHardLimits-request.msg.html

(cl:defclass <GetKinematicHardLimits-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetKinematicHardLimits-request (<GetKinematicHardLimits-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetKinematicHardLimits-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetKinematicHardLimits-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetKinematicHardLimits-request> is deprecated: use kortex_driver-srv:GetKinematicHardLimits-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetKinematicHardLimits-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetKinematicHardLimits-request>) ostream)
  "Serializes a message object of type '<GetKinematicHardLimits-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetKinematicHardLimits-request>) istream)
  "Deserializes a message object of type '<GetKinematicHardLimits-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetKinematicHardLimits-request>)))
  "Returns string type for a service object of type '<GetKinematicHardLimits-request>"
  "kortex_driver/GetKinematicHardLimitsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetKinematicHardLimits-request)))
  "Returns string type for a service object of type 'GetKinematicHardLimits-request"
  "kortex_driver/GetKinematicHardLimitsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetKinematicHardLimits-request>)))
  "Returns md5sum for a message object of type '<GetKinematicHardLimits-request>"
  "024037c761d64c76474f3dff50b971a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetKinematicHardLimits-request)))
  "Returns md5sum for a message object of type 'GetKinematicHardLimits-request"
  "024037c761d64c76474f3dff50b971a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetKinematicHardLimits-request>)))
  "Returns full string definition for message of type '<GetKinematicHardLimits-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetKinematicHardLimits-request)))
  "Returns full string definition for message of type 'GetKinematicHardLimits-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetKinematicHardLimits-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetKinematicHardLimits-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetKinematicHardLimits-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetKinematicHardLimits-response.msg.html

(cl:defclass <GetKinematicHardLimits-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:KinematicLimits
    :initform (cl:make-instance 'kortex_driver-msg:KinematicLimits)))
)

(cl:defclass GetKinematicHardLimits-response (<GetKinematicHardLimits-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetKinematicHardLimits-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetKinematicHardLimits-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetKinematicHardLimits-response> is deprecated: use kortex_driver-srv:GetKinematicHardLimits-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetKinematicHardLimits-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetKinematicHardLimits-response>) ostream)
  "Serializes a message object of type '<GetKinematicHardLimits-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetKinematicHardLimits-response>) istream)
  "Deserializes a message object of type '<GetKinematicHardLimits-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetKinematicHardLimits-response>)))
  "Returns string type for a service object of type '<GetKinematicHardLimits-response>"
  "kortex_driver/GetKinematicHardLimitsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetKinematicHardLimits-response)))
  "Returns string type for a service object of type 'GetKinematicHardLimits-response"
  "kortex_driver/GetKinematicHardLimitsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetKinematicHardLimits-response>)))
  "Returns md5sum for a message object of type '<GetKinematicHardLimits-response>"
  "024037c761d64c76474f3dff50b971a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetKinematicHardLimits-response)))
  "Returns md5sum for a message object of type 'GetKinematicHardLimits-response"
  "024037c761d64c76474f3dff50b971a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetKinematicHardLimits-response>)))
  "Returns full string definition for message of type '<GetKinematicHardLimits-response>"
  (cl:format cl:nil "KinematicLimits output~%~%================================================================================~%MSG: kortex_driver/KinematicLimits~%~%uint32 control_mode~%float32 twist_linear~%float32 twist_angular~%float32[] joint_speed_limits~%float32[] joint_acceleration_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetKinematicHardLimits-response)))
  "Returns full string definition for message of type 'GetKinematicHardLimits-response"
  (cl:format cl:nil "KinematicLimits output~%~%================================================================================~%MSG: kortex_driver/KinematicLimits~%~%uint32 control_mode~%float32 twist_linear~%float32 twist_angular~%float32[] joint_speed_limits~%float32[] joint_acceleration_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetKinematicHardLimits-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetKinematicHardLimits-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetKinematicHardLimits-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetKinematicHardLimits)))
  'GetKinematicHardLimits-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetKinematicHardLimits)))
  'GetKinematicHardLimits-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetKinematicHardLimits)))
  "Returns string type for a service object of type '<GetKinematicHardLimits>"
  "kortex_driver/GetKinematicHardLimits")