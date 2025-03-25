; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetDesiredJointSpeeds-request.msg.html

(cl:defclass <SetDesiredJointSpeeds-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ControlConfig_JointSpeeds
    :initform (cl:make-instance 'kortex_driver-msg:ControlConfig_JointSpeeds)))
)

(cl:defclass SetDesiredJointSpeeds-request (<SetDesiredJointSpeeds-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDesiredJointSpeeds-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDesiredJointSpeeds-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetDesiredJointSpeeds-request> is deprecated: use kortex_driver-srv:SetDesiredJointSpeeds-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetDesiredJointSpeeds-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDesiredJointSpeeds-request>) ostream)
  "Serializes a message object of type '<SetDesiredJointSpeeds-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDesiredJointSpeeds-request>) istream)
  "Deserializes a message object of type '<SetDesiredJointSpeeds-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDesiredJointSpeeds-request>)))
  "Returns string type for a service object of type '<SetDesiredJointSpeeds-request>"
  "kortex_driver/SetDesiredJointSpeedsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDesiredJointSpeeds-request)))
  "Returns string type for a service object of type 'SetDesiredJointSpeeds-request"
  "kortex_driver/SetDesiredJointSpeedsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDesiredJointSpeeds-request>)))
  "Returns md5sum for a message object of type '<SetDesiredJointSpeeds-request>"
  "57cd70ec5519968c6e959068a3575705")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDesiredJointSpeeds-request)))
  "Returns md5sum for a message object of type 'SetDesiredJointSpeeds-request"
  "57cd70ec5519968c6e959068a3575705")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDesiredJointSpeeds-request>)))
  "Returns full string definition for message of type '<SetDesiredJointSpeeds-request>"
  (cl:format cl:nil "ControlConfig_JointSpeeds input~%~%================================================================================~%MSG: kortex_driver/ControlConfig_JointSpeeds~%~%float32[] joint_speed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDesiredJointSpeeds-request)))
  "Returns full string definition for message of type 'SetDesiredJointSpeeds-request"
  (cl:format cl:nil "ControlConfig_JointSpeeds input~%~%================================================================================~%MSG: kortex_driver/ControlConfig_JointSpeeds~%~%float32[] joint_speed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDesiredJointSpeeds-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDesiredJointSpeeds-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDesiredJointSpeeds-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetDesiredJointSpeeds-response.msg.html

(cl:defclass <SetDesiredJointSpeeds-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetDesiredJointSpeeds-response (<SetDesiredJointSpeeds-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDesiredJointSpeeds-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDesiredJointSpeeds-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetDesiredJointSpeeds-response> is deprecated: use kortex_driver-srv:SetDesiredJointSpeeds-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetDesiredJointSpeeds-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDesiredJointSpeeds-response>) ostream)
  "Serializes a message object of type '<SetDesiredJointSpeeds-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDesiredJointSpeeds-response>) istream)
  "Deserializes a message object of type '<SetDesiredJointSpeeds-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDesiredJointSpeeds-response>)))
  "Returns string type for a service object of type '<SetDesiredJointSpeeds-response>"
  "kortex_driver/SetDesiredJointSpeedsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDesiredJointSpeeds-response)))
  "Returns string type for a service object of type 'SetDesiredJointSpeeds-response"
  "kortex_driver/SetDesiredJointSpeedsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDesiredJointSpeeds-response>)))
  "Returns md5sum for a message object of type '<SetDesiredJointSpeeds-response>"
  "57cd70ec5519968c6e959068a3575705")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDesiredJointSpeeds-response)))
  "Returns md5sum for a message object of type 'SetDesiredJointSpeeds-response"
  "57cd70ec5519968c6e959068a3575705")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDesiredJointSpeeds-response>)))
  "Returns full string definition for message of type '<SetDesiredJointSpeeds-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDesiredJointSpeeds-response)))
  "Returns full string definition for message of type 'SetDesiredJointSpeeds-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDesiredJointSpeeds-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDesiredJointSpeeds-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDesiredJointSpeeds-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetDesiredJointSpeeds)))
  'SetDesiredJointSpeeds-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetDesiredJointSpeeds)))
  'SetDesiredJointSpeeds-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDesiredJointSpeeds)))
  "Returns string type for a service object of type '<SetDesiredJointSpeeds>"
  "kortex_driver/SetDesiredJointSpeeds")