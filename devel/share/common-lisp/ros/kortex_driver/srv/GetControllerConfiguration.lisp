; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetControllerConfiguration-request.msg.html

(cl:defclass <GetControllerConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ControllerHandle
    :initform (cl:make-instance 'kortex_driver-msg:ControllerHandle)))
)

(cl:defclass GetControllerConfiguration-request (<GetControllerConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetControllerConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetControllerConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetControllerConfiguration-request> is deprecated: use kortex_driver-srv:GetControllerConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetControllerConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetControllerConfiguration-request>) ostream)
  "Serializes a message object of type '<GetControllerConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetControllerConfiguration-request>) istream)
  "Deserializes a message object of type '<GetControllerConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetControllerConfiguration-request>)))
  "Returns string type for a service object of type '<GetControllerConfiguration-request>"
  "kortex_driver/GetControllerConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControllerConfiguration-request)))
  "Returns string type for a service object of type 'GetControllerConfiguration-request"
  "kortex_driver/GetControllerConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetControllerConfiguration-request>)))
  "Returns md5sum for a message object of type '<GetControllerConfiguration-request>"
  "486c0887673fbfba27c914b77b97b479")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetControllerConfiguration-request)))
  "Returns md5sum for a message object of type 'GetControllerConfiguration-request"
  "486c0887673fbfba27c914b77b97b479")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetControllerConfiguration-request>)))
  "Returns full string definition for message of type '<GetControllerConfiguration-request>"
  (cl:format cl:nil "ControllerHandle input~%~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetControllerConfiguration-request)))
  "Returns full string definition for message of type 'GetControllerConfiguration-request"
  (cl:format cl:nil "ControllerHandle input~%~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetControllerConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetControllerConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetControllerConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetControllerConfiguration-response.msg.html

(cl:defclass <GetControllerConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:ControllerConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:ControllerConfiguration)))
)

(cl:defclass GetControllerConfiguration-response (<GetControllerConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetControllerConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetControllerConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetControllerConfiguration-response> is deprecated: use kortex_driver-srv:GetControllerConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetControllerConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetControllerConfiguration-response>) ostream)
  "Serializes a message object of type '<GetControllerConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetControllerConfiguration-response>) istream)
  "Deserializes a message object of type '<GetControllerConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetControllerConfiguration-response>)))
  "Returns string type for a service object of type '<GetControllerConfiguration-response>"
  "kortex_driver/GetControllerConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControllerConfiguration-response)))
  "Returns string type for a service object of type 'GetControllerConfiguration-response"
  "kortex_driver/GetControllerConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetControllerConfiguration-response>)))
  "Returns md5sum for a message object of type '<GetControllerConfiguration-response>"
  "486c0887673fbfba27c914b77b97b479")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetControllerConfiguration-response)))
  "Returns md5sum for a message object of type 'GetControllerConfiguration-response"
  "486c0887673fbfba27c914b77b97b479")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetControllerConfiguration-response>)))
  "Returns full string definition for message of type '<GetControllerConfiguration-response>"
  (cl:format cl:nil "ControllerConfiguration output~%~%================================================================================~%MSG: kortex_driver/ControllerConfiguration~%~%ControllerHandle handle~%string name~%MappingHandle active_mapping_handle~%string analog_input_identifier_enum~%string digital_input_identifier_enum~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetControllerConfiguration-response)))
  "Returns full string definition for message of type 'GetControllerConfiguration-response"
  (cl:format cl:nil "ControllerConfiguration output~%~%================================================================================~%MSG: kortex_driver/ControllerConfiguration~%~%ControllerHandle handle~%string name~%MappingHandle active_mapping_handle~%string analog_input_identifier_enum~%string digital_input_identifier_enum~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetControllerConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetControllerConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetControllerConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetControllerConfiguration)))
  'GetControllerConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetControllerConfiguration)))
  'GetControllerConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControllerConfiguration)))
  "Returns string type for a service object of type '<GetControllerConfiguration>"
  "kortex_driver/GetControllerConfiguration")