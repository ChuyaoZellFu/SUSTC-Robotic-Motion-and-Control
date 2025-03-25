; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetControlMode-request.msg.html

(cl:defclass <SetControlMode-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ActuatorConfig_ControlModeInformation
    :initform (cl:make-instance 'kortex_driver-msg:ActuatorConfig_ControlModeInformation)))
)

(cl:defclass SetControlMode-request (<SetControlMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetControlMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetControlMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetControlMode-request> is deprecated: use kortex_driver-srv:SetControlMode-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetControlMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetControlMode-request>) ostream)
  "Serializes a message object of type '<SetControlMode-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetControlMode-request>) istream)
  "Deserializes a message object of type '<SetControlMode-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetControlMode-request>)))
  "Returns string type for a service object of type '<SetControlMode-request>"
  "kortex_driver/SetControlModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControlMode-request)))
  "Returns string type for a service object of type 'SetControlMode-request"
  "kortex_driver/SetControlModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetControlMode-request>)))
  "Returns md5sum for a message object of type '<SetControlMode-request>"
  "cd48317b2c3b2a87721b906a23a31b2f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetControlMode-request)))
  "Returns md5sum for a message object of type 'SetControlMode-request"
  "cd48317b2c3b2a87721b906a23a31b2f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetControlMode-request>)))
  "Returns full string definition for message of type '<SetControlMode-request>"
  (cl:format cl:nil "ActuatorConfig_ControlModeInformation input~%~%================================================================================~%MSG: kortex_driver/ActuatorConfig_ControlModeInformation~%~%uint32 control_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetControlMode-request)))
  "Returns full string definition for message of type 'SetControlMode-request"
  (cl:format cl:nil "ActuatorConfig_ControlModeInformation input~%~%================================================================================~%MSG: kortex_driver/ActuatorConfig_ControlModeInformation~%~%uint32 control_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetControlMode-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetControlMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetControlMode-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetControlMode-response.msg.html

(cl:defclass <SetControlMode-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetControlMode-response (<SetControlMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetControlMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetControlMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetControlMode-response> is deprecated: use kortex_driver-srv:SetControlMode-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetControlMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetControlMode-response>) ostream)
  "Serializes a message object of type '<SetControlMode-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetControlMode-response>) istream)
  "Deserializes a message object of type '<SetControlMode-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetControlMode-response>)))
  "Returns string type for a service object of type '<SetControlMode-response>"
  "kortex_driver/SetControlModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControlMode-response)))
  "Returns string type for a service object of type 'SetControlMode-response"
  "kortex_driver/SetControlModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetControlMode-response>)))
  "Returns md5sum for a message object of type '<SetControlMode-response>"
  "cd48317b2c3b2a87721b906a23a31b2f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetControlMode-response)))
  "Returns md5sum for a message object of type 'SetControlMode-response"
  "cd48317b2c3b2a87721b906a23a31b2f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetControlMode-response>)))
  "Returns full string definition for message of type '<SetControlMode-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetControlMode-response)))
  "Returns full string definition for message of type 'SetControlMode-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetControlMode-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetControlMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetControlMode-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetControlMode)))
  'SetControlMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetControlMode)))
  'SetControlMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControlMode)))
  "Returns string type for a service object of type '<SetControlMode>"
  "kortex_driver/SetControlMode")