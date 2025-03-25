; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetAllControllerConfigurations-request.msg.html

(cl:defclass <GetAllControllerConfigurations-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetAllControllerConfigurations-request (<GetAllControllerConfigurations-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllControllerConfigurations-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllControllerConfigurations-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllControllerConfigurations-request> is deprecated: use kortex_driver-srv:GetAllControllerConfigurations-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetAllControllerConfigurations-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllControllerConfigurations-request>) ostream)
  "Serializes a message object of type '<GetAllControllerConfigurations-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllControllerConfigurations-request>) istream)
  "Deserializes a message object of type '<GetAllControllerConfigurations-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllControllerConfigurations-request>)))
  "Returns string type for a service object of type '<GetAllControllerConfigurations-request>"
  "kortex_driver/GetAllControllerConfigurationsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllControllerConfigurations-request)))
  "Returns string type for a service object of type 'GetAllControllerConfigurations-request"
  "kortex_driver/GetAllControllerConfigurationsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllControllerConfigurations-request>)))
  "Returns md5sum for a message object of type '<GetAllControllerConfigurations-request>"
  "48bc932dac09542b910dbdc1482df5ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllControllerConfigurations-request)))
  "Returns md5sum for a message object of type 'GetAllControllerConfigurations-request"
  "48bc932dac09542b910dbdc1482df5ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllControllerConfigurations-request>)))
  "Returns full string definition for message of type '<GetAllControllerConfigurations-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllControllerConfigurations-request)))
  "Returns full string definition for message of type 'GetAllControllerConfigurations-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllControllerConfigurations-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllControllerConfigurations-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllControllerConfigurations-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetAllControllerConfigurations-response.msg.html

(cl:defclass <GetAllControllerConfigurations-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:ControllerConfigurationList
    :initform (cl:make-instance 'kortex_driver-msg:ControllerConfigurationList)))
)

(cl:defclass GetAllControllerConfigurations-response (<GetAllControllerConfigurations-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllControllerConfigurations-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllControllerConfigurations-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllControllerConfigurations-response> is deprecated: use kortex_driver-srv:GetAllControllerConfigurations-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetAllControllerConfigurations-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllControllerConfigurations-response>) ostream)
  "Serializes a message object of type '<GetAllControllerConfigurations-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllControllerConfigurations-response>) istream)
  "Deserializes a message object of type '<GetAllControllerConfigurations-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllControllerConfigurations-response>)))
  "Returns string type for a service object of type '<GetAllControllerConfigurations-response>"
  "kortex_driver/GetAllControllerConfigurationsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllControllerConfigurations-response)))
  "Returns string type for a service object of type 'GetAllControllerConfigurations-response"
  "kortex_driver/GetAllControllerConfigurationsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllControllerConfigurations-response>)))
  "Returns md5sum for a message object of type '<GetAllControllerConfigurations-response>"
  "48bc932dac09542b910dbdc1482df5ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllControllerConfigurations-response)))
  "Returns md5sum for a message object of type 'GetAllControllerConfigurations-response"
  "48bc932dac09542b910dbdc1482df5ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllControllerConfigurations-response>)))
  "Returns full string definition for message of type '<GetAllControllerConfigurations-response>"
  (cl:format cl:nil "ControllerConfigurationList output~%~%================================================================================~%MSG: kortex_driver/ControllerConfigurationList~%~%ControllerConfiguration[] controller_configurations~%================================================================================~%MSG: kortex_driver/ControllerConfiguration~%~%ControllerHandle handle~%string name~%MappingHandle active_mapping_handle~%string analog_input_identifier_enum~%string digital_input_identifier_enum~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllControllerConfigurations-response)))
  "Returns full string definition for message of type 'GetAllControllerConfigurations-response"
  (cl:format cl:nil "ControllerConfigurationList output~%~%================================================================================~%MSG: kortex_driver/ControllerConfigurationList~%~%ControllerConfiguration[] controller_configurations~%================================================================================~%MSG: kortex_driver/ControllerConfiguration~%~%ControllerHandle handle~%string name~%MappingHandle active_mapping_handle~%string analog_input_identifier_enum~%string digital_input_identifier_enum~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllControllerConfigurations-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllControllerConfigurations-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllControllerConfigurations-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAllControllerConfigurations)))
  'GetAllControllerConfigurations-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAllControllerConfigurations)))
  'GetAllControllerConfigurations-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllControllerConfigurations)))
  "Returns string type for a service object of type '<GetAllControllerConfigurations>"
  "kortex_driver/GetAllControllerConfigurations")