; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetToolConfiguration-request.msg.html

(cl:defclass <SetToolConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ToolConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:ToolConfiguration)))
)

(cl:defclass SetToolConfiguration-request (<SetToolConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetToolConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetToolConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetToolConfiguration-request> is deprecated: use kortex_driver-srv:SetToolConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetToolConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetToolConfiguration-request>) ostream)
  "Serializes a message object of type '<SetToolConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetToolConfiguration-request>) istream)
  "Deserializes a message object of type '<SetToolConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetToolConfiguration-request>)))
  "Returns string type for a service object of type '<SetToolConfiguration-request>"
  "kortex_driver/SetToolConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetToolConfiguration-request)))
  "Returns string type for a service object of type 'SetToolConfiguration-request"
  "kortex_driver/SetToolConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetToolConfiguration-request>)))
  "Returns md5sum for a message object of type '<SetToolConfiguration-request>"
  "d0f38c8ed77e0310c120105db58e4900")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetToolConfiguration-request)))
  "Returns md5sum for a message object of type 'SetToolConfiguration-request"
  "d0f38c8ed77e0310c120105db58e4900")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetToolConfiguration-request>)))
  "Returns full string definition for message of type '<SetToolConfiguration-request>"
  (cl:format cl:nil "ToolConfiguration input~%~%================================================================================~%MSG: kortex_driver/ToolConfiguration~%~%CartesianTransform tool_transform~%float32 tool_mass~%ControlConfig_Position tool_mass_center~%================================================================================~%MSG: kortex_driver/CartesianTransform~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/ControlConfig_Position~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetToolConfiguration-request)))
  "Returns full string definition for message of type 'SetToolConfiguration-request"
  (cl:format cl:nil "ToolConfiguration input~%~%================================================================================~%MSG: kortex_driver/ToolConfiguration~%~%CartesianTransform tool_transform~%float32 tool_mass~%ControlConfig_Position tool_mass_center~%================================================================================~%MSG: kortex_driver/CartesianTransform~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/ControlConfig_Position~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetToolConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetToolConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetToolConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetToolConfiguration-response.msg.html

(cl:defclass <SetToolConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetToolConfiguration-response (<SetToolConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetToolConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetToolConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetToolConfiguration-response> is deprecated: use kortex_driver-srv:SetToolConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetToolConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetToolConfiguration-response>) ostream)
  "Serializes a message object of type '<SetToolConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetToolConfiguration-response>) istream)
  "Deserializes a message object of type '<SetToolConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetToolConfiguration-response>)))
  "Returns string type for a service object of type '<SetToolConfiguration-response>"
  "kortex_driver/SetToolConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetToolConfiguration-response)))
  "Returns string type for a service object of type 'SetToolConfiguration-response"
  "kortex_driver/SetToolConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetToolConfiguration-response>)))
  "Returns md5sum for a message object of type '<SetToolConfiguration-response>"
  "d0f38c8ed77e0310c120105db58e4900")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetToolConfiguration-response)))
  "Returns md5sum for a message object of type 'SetToolConfiguration-response"
  "d0f38c8ed77e0310c120105db58e4900")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetToolConfiguration-response>)))
  "Returns full string definition for message of type '<SetToolConfiguration-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetToolConfiguration-response)))
  "Returns full string definition for message of type 'SetToolConfiguration-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetToolConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetToolConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetToolConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetToolConfiguration)))
  'SetToolConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetToolConfiguration)))
  'SetToolConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetToolConfiguration)))
  "Returns string type for a service object of type '<SetToolConfiguration>"
  "kortex_driver/SetToolConfiguration")