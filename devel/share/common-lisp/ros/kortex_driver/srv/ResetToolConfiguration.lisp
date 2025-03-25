; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ResetToolConfiguration-request.msg.html

(cl:defclass <ResetToolConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ResetToolConfiguration-request (<ResetToolConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetToolConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetToolConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ResetToolConfiguration-request> is deprecated: use kortex_driver-srv:ResetToolConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ResetToolConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetToolConfiguration-request>) ostream)
  "Serializes a message object of type '<ResetToolConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetToolConfiguration-request>) istream)
  "Deserializes a message object of type '<ResetToolConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetToolConfiguration-request>)))
  "Returns string type for a service object of type '<ResetToolConfiguration-request>"
  "kortex_driver/ResetToolConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetToolConfiguration-request)))
  "Returns string type for a service object of type 'ResetToolConfiguration-request"
  "kortex_driver/ResetToolConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetToolConfiguration-request>)))
  "Returns md5sum for a message object of type '<ResetToolConfiguration-request>"
  "49c06fd31c2188ee0d629d1b8491b1f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetToolConfiguration-request)))
  "Returns md5sum for a message object of type 'ResetToolConfiguration-request"
  "49c06fd31c2188ee0d629d1b8491b1f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetToolConfiguration-request>)))
  "Returns full string definition for message of type '<ResetToolConfiguration-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetToolConfiguration-request)))
  "Returns full string definition for message of type 'ResetToolConfiguration-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetToolConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetToolConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetToolConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ResetToolConfiguration-response.msg.html

(cl:defclass <ResetToolConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:ToolConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:ToolConfiguration)))
)

(cl:defclass ResetToolConfiguration-response (<ResetToolConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetToolConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetToolConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ResetToolConfiguration-response> is deprecated: use kortex_driver-srv:ResetToolConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ResetToolConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetToolConfiguration-response>) ostream)
  "Serializes a message object of type '<ResetToolConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetToolConfiguration-response>) istream)
  "Deserializes a message object of type '<ResetToolConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetToolConfiguration-response>)))
  "Returns string type for a service object of type '<ResetToolConfiguration-response>"
  "kortex_driver/ResetToolConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetToolConfiguration-response)))
  "Returns string type for a service object of type 'ResetToolConfiguration-response"
  "kortex_driver/ResetToolConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetToolConfiguration-response>)))
  "Returns md5sum for a message object of type '<ResetToolConfiguration-response>"
  "49c06fd31c2188ee0d629d1b8491b1f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetToolConfiguration-response)))
  "Returns md5sum for a message object of type 'ResetToolConfiguration-response"
  "49c06fd31c2188ee0d629d1b8491b1f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetToolConfiguration-response>)))
  "Returns full string definition for message of type '<ResetToolConfiguration-response>"
  (cl:format cl:nil "ToolConfiguration output~%~%================================================================================~%MSG: kortex_driver/ToolConfiguration~%~%CartesianTransform tool_transform~%float32 tool_mass~%ControlConfig_Position tool_mass_center~%================================================================================~%MSG: kortex_driver/CartesianTransform~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/ControlConfig_Position~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetToolConfiguration-response)))
  "Returns full string definition for message of type 'ResetToolConfiguration-response"
  (cl:format cl:nil "ToolConfiguration output~%~%================================================================================~%MSG: kortex_driver/ToolConfiguration~%~%CartesianTransform tool_transform~%float32 tool_mass~%ControlConfig_Position tool_mass_center~%================================================================================~%MSG: kortex_driver/CartesianTransform~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/ControlConfig_Position~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetToolConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetToolConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetToolConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetToolConfiguration)))
  'ResetToolConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetToolConfiguration)))
  'ResetToolConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetToolConfiguration)))
  "Returns string type for a service object of type '<ResetToolConfiguration>"
  "kortex_driver/ResetToolConfiguration")