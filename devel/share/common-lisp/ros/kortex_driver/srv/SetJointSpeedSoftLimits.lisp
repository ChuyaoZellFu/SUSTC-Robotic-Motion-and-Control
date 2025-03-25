; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetJointSpeedSoftLimits-request.msg.html

(cl:defclass <SetJointSpeedSoftLimits-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:JointSpeedSoftLimits
    :initform (cl:make-instance 'kortex_driver-msg:JointSpeedSoftLimits)))
)

(cl:defclass SetJointSpeedSoftLimits-request (<SetJointSpeedSoftLimits-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointSpeedSoftLimits-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointSpeedSoftLimits-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetJointSpeedSoftLimits-request> is deprecated: use kortex_driver-srv:SetJointSpeedSoftLimits-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetJointSpeedSoftLimits-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointSpeedSoftLimits-request>) ostream)
  "Serializes a message object of type '<SetJointSpeedSoftLimits-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointSpeedSoftLimits-request>) istream)
  "Deserializes a message object of type '<SetJointSpeedSoftLimits-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointSpeedSoftLimits-request>)))
  "Returns string type for a service object of type '<SetJointSpeedSoftLimits-request>"
  "kortex_driver/SetJointSpeedSoftLimitsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointSpeedSoftLimits-request)))
  "Returns string type for a service object of type 'SetJointSpeedSoftLimits-request"
  "kortex_driver/SetJointSpeedSoftLimitsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointSpeedSoftLimits-request>)))
  "Returns md5sum for a message object of type '<SetJointSpeedSoftLimits-request>"
  "f67059c3b3849506a74d17610ccdb240")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointSpeedSoftLimits-request)))
  "Returns md5sum for a message object of type 'SetJointSpeedSoftLimits-request"
  "f67059c3b3849506a74d17610ccdb240")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointSpeedSoftLimits-request>)))
  "Returns full string definition for message of type '<SetJointSpeedSoftLimits-request>"
  (cl:format cl:nil "JointSpeedSoftLimits input~%~%================================================================================~%MSG: kortex_driver/JointSpeedSoftLimits~%~%uint32 control_mode~%float32[] joint_speed_soft_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointSpeedSoftLimits-request)))
  "Returns full string definition for message of type 'SetJointSpeedSoftLimits-request"
  (cl:format cl:nil "JointSpeedSoftLimits input~%~%================================================================================~%MSG: kortex_driver/JointSpeedSoftLimits~%~%uint32 control_mode~%float32[] joint_speed_soft_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointSpeedSoftLimits-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointSpeedSoftLimits-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointSpeedSoftLimits-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetJointSpeedSoftLimits-response.msg.html

(cl:defclass <SetJointSpeedSoftLimits-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetJointSpeedSoftLimits-response (<SetJointSpeedSoftLimits-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointSpeedSoftLimits-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointSpeedSoftLimits-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetJointSpeedSoftLimits-response> is deprecated: use kortex_driver-srv:SetJointSpeedSoftLimits-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetJointSpeedSoftLimits-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointSpeedSoftLimits-response>) ostream)
  "Serializes a message object of type '<SetJointSpeedSoftLimits-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointSpeedSoftLimits-response>) istream)
  "Deserializes a message object of type '<SetJointSpeedSoftLimits-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointSpeedSoftLimits-response>)))
  "Returns string type for a service object of type '<SetJointSpeedSoftLimits-response>"
  "kortex_driver/SetJointSpeedSoftLimitsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointSpeedSoftLimits-response)))
  "Returns string type for a service object of type 'SetJointSpeedSoftLimits-response"
  "kortex_driver/SetJointSpeedSoftLimitsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointSpeedSoftLimits-response>)))
  "Returns md5sum for a message object of type '<SetJointSpeedSoftLimits-response>"
  "f67059c3b3849506a74d17610ccdb240")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointSpeedSoftLimits-response)))
  "Returns md5sum for a message object of type 'SetJointSpeedSoftLimits-response"
  "f67059c3b3849506a74d17610ccdb240")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointSpeedSoftLimits-response>)))
  "Returns full string definition for message of type '<SetJointSpeedSoftLimits-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointSpeedSoftLimits-response)))
  "Returns full string definition for message of type 'SetJointSpeedSoftLimits-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointSpeedSoftLimits-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointSpeedSoftLimits-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointSpeedSoftLimits-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetJointSpeedSoftLimits)))
  'SetJointSpeedSoftLimits-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetJointSpeedSoftLimits)))
  'SetJointSpeedSoftLimits-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointSpeedSoftLimits)))
  "Returns string type for a service object of type '<SetJointSpeedSoftLimits>"
  "kortex_driver/SetJointSpeedSoftLimits")