; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetJointAccelerationSoftLimits-request.msg.html

(cl:defclass <SetJointAccelerationSoftLimits-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:JointAccelerationSoftLimits
    :initform (cl:make-instance 'kortex_driver-msg:JointAccelerationSoftLimits)))
)

(cl:defclass SetJointAccelerationSoftLimits-request (<SetJointAccelerationSoftLimits-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointAccelerationSoftLimits-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointAccelerationSoftLimits-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetJointAccelerationSoftLimits-request> is deprecated: use kortex_driver-srv:SetJointAccelerationSoftLimits-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetJointAccelerationSoftLimits-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointAccelerationSoftLimits-request>) ostream)
  "Serializes a message object of type '<SetJointAccelerationSoftLimits-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointAccelerationSoftLimits-request>) istream)
  "Deserializes a message object of type '<SetJointAccelerationSoftLimits-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointAccelerationSoftLimits-request>)))
  "Returns string type for a service object of type '<SetJointAccelerationSoftLimits-request>"
  "kortex_driver/SetJointAccelerationSoftLimitsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointAccelerationSoftLimits-request)))
  "Returns string type for a service object of type 'SetJointAccelerationSoftLimits-request"
  "kortex_driver/SetJointAccelerationSoftLimitsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointAccelerationSoftLimits-request>)))
  "Returns md5sum for a message object of type '<SetJointAccelerationSoftLimits-request>"
  "cee0ecb0e446175f3b5cec2dbc1076e5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointAccelerationSoftLimits-request)))
  "Returns md5sum for a message object of type 'SetJointAccelerationSoftLimits-request"
  "cee0ecb0e446175f3b5cec2dbc1076e5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointAccelerationSoftLimits-request>)))
  "Returns full string definition for message of type '<SetJointAccelerationSoftLimits-request>"
  (cl:format cl:nil "JointAccelerationSoftLimits input~%~%================================================================================~%MSG: kortex_driver/JointAccelerationSoftLimits~%~%uint32 control_mode~%float32[] joint_acceleration_soft_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointAccelerationSoftLimits-request)))
  "Returns full string definition for message of type 'SetJointAccelerationSoftLimits-request"
  (cl:format cl:nil "JointAccelerationSoftLimits input~%~%================================================================================~%MSG: kortex_driver/JointAccelerationSoftLimits~%~%uint32 control_mode~%float32[] joint_acceleration_soft_limits~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointAccelerationSoftLimits-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointAccelerationSoftLimits-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointAccelerationSoftLimits-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetJointAccelerationSoftLimits-response.msg.html

(cl:defclass <SetJointAccelerationSoftLimits-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetJointAccelerationSoftLimits-response (<SetJointAccelerationSoftLimits-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointAccelerationSoftLimits-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointAccelerationSoftLimits-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetJointAccelerationSoftLimits-response> is deprecated: use kortex_driver-srv:SetJointAccelerationSoftLimits-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetJointAccelerationSoftLimits-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointAccelerationSoftLimits-response>) ostream)
  "Serializes a message object of type '<SetJointAccelerationSoftLimits-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointAccelerationSoftLimits-response>) istream)
  "Deserializes a message object of type '<SetJointAccelerationSoftLimits-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointAccelerationSoftLimits-response>)))
  "Returns string type for a service object of type '<SetJointAccelerationSoftLimits-response>"
  "kortex_driver/SetJointAccelerationSoftLimitsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointAccelerationSoftLimits-response)))
  "Returns string type for a service object of type 'SetJointAccelerationSoftLimits-response"
  "kortex_driver/SetJointAccelerationSoftLimitsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointAccelerationSoftLimits-response>)))
  "Returns md5sum for a message object of type '<SetJointAccelerationSoftLimits-response>"
  "cee0ecb0e446175f3b5cec2dbc1076e5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointAccelerationSoftLimits-response)))
  "Returns md5sum for a message object of type 'SetJointAccelerationSoftLimits-response"
  "cee0ecb0e446175f3b5cec2dbc1076e5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointAccelerationSoftLimits-response>)))
  "Returns full string definition for message of type '<SetJointAccelerationSoftLimits-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointAccelerationSoftLimits-response)))
  "Returns full string definition for message of type 'SetJointAccelerationSoftLimits-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointAccelerationSoftLimits-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointAccelerationSoftLimits-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointAccelerationSoftLimits-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetJointAccelerationSoftLimits)))
  'SetJointAccelerationSoftLimits-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetJointAccelerationSoftLimits)))
  'SetJointAccelerationSoftLimits-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointAccelerationSoftLimits)))
  "Returns string type for a service object of type '<SetJointAccelerationSoftLimits>"
  "kortex_driver/SetJointAccelerationSoftLimits")