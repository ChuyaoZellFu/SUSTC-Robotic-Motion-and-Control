; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetToolConfiguration-request.msg.html

(cl:defclass <GetToolConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetToolConfiguration-request (<GetToolConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetToolConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetToolConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetToolConfiguration-request> is deprecated: use kortex_driver-srv:GetToolConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetToolConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetToolConfiguration-request>) ostream)
  "Serializes a message object of type '<GetToolConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetToolConfiguration-request>) istream)
  "Deserializes a message object of type '<GetToolConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetToolConfiguration-request>)))
  "Returns string type for a service object of type '<GetToolConfiguration-request>"
  "kortex_driver/GetToolConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetToolConfiguration-request)))
  "Returns string type for a service object of type 'GetToolConfiguration-request"
  "kortex_driver/GetToolConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetToolConfiguration-request>)))
  "Returns md5sum for a message object of type '<GetToolConfiguration-request>"
  "49c06fd31c2188ee0d629d1b8491b1f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetToolConfiguration-request)))
  "Returns md5sum for a message object of type 'GetToolConfiguration-request"
  "49c06fd31c2188ee0d629d1b8491b1f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetToolConfiguration-request>)))
  "Returns full string definition for message of type '<GetToolConfiguration-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetToolConfiguration-request)))
  "Returns full string definition for message of type 'GetToolConfiguration-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetToolConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetToolConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetToolConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetToolConfiguration-response.msg.html

(cl:defclass <GetToolConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:ToolConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:ToolConfiguration)))
)

(cl:defclass GetToolConfiguration-response (<GetToolConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetToolConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetToolConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetToolConfiguration-response> is deprecated: use kortex_driver-srv:GetToolConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetToolConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetToolConfiguration-response>) ostream)
  "Serializes a message object of type '<GetToolConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetToolConfiguration-response>) istream)
  "Deserializes a message object of type '<GetToolConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetToolConfiguration-response>)))
  "Returns string type for a service object of type '<GetToolConfiguration-response>"
  "kortex_driver/GetToolConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetToolConfiguration-response)))
  "Returns string type for a service object of type 'GetToolConfiguration-response"
  "kortex_driver/GetToolConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetToolConfiguration-response>)))
  "Returns md5sum for a message object of type '<GetToolConfiguration-response>"
  "49c06fd31c2188ee0d629d1b8491b1f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetToolConfiguration-response)))
  "Returns md5sum for a message object of type 'GetToolConfiguration-response"
  "49c06fd31c2188ee0d629d1b8491b1f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetToolConfiguration-response>)))
  "Returns full string definition for message of type '<GetToolConfiguration-response>"
  (cl:format cl:nil "ToolConfiguration output~%~%================================================================================~%MSG: kortex_driver/ToolConfiguration~%~%CartesianTransform tool_transform~%float32 tool_mass~%ControlConfig_Position tool_mass_center~%================================================================================~%MSG: kortex_driver/CartesianTransform~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/ControlConfig_Position~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetToolConfiguration-response)))
  "Returns full string definition for message of type 'GetToolConfiguration-response"
  (cl:format cl:nil "ToolConfiguration output~%~%================================================================================~%MSG: kortex_driver/ToolConfiguration~%~%CartesianTransform tool_transform~%float32 tool_mass~%ControlConfig_Position tool_mass_center~%================================================================================~%MSG: kortex_driver/CartesianTransform~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/ControlConfig_Position~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetToolConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetToolConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetToolConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetToolConfiguration)))
  'GetToolConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetToolConfiguration)))
  'GetToolConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetToolConfiguration)))
  "Returns string type for a service object of type '<GetToolConfiguration>"
  "kortex_driver/GetToolConfiguration")