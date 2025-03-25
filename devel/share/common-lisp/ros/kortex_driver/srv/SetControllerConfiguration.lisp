; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetControllerConfiguration-request.msg.html

(cl:defclass <SetControllerConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ControllerConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:ControllerConfiguration)))
)

(cl:defclass SetControllerConfiguration-request (<SetControllerConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetControllerConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetControllerConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetControllerConfiguration-request> is deprecated: use kortex_driver-srv:SetControllerConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetControllerConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetControllerConfiguration-request>) ostream)
  "Serializes a message object of type '<SetControllerConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetControllerConfiguration-request>) istream)
  "Deserializes a message object of type '<SetControllerConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetControllerConfiguration-request>)))
  "Returns string type for a service object of type '<SetControllerConfiguration-request>"
  "kortex_driver/SetControllerConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControllerConfiguration-request)))
  "Returns string type for a service object of type 'SetControllerConfiguration-request"
  "kortex_driver/SetControllerConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetControllerConfiguration-request>)))
  "Returns md5sum for a message object of type '<SetControllerConfiguration-request>"
  "4158c0b1ce15d5824908b5157fccd994")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetControllerConfiguration-request)))
  "Returns md5sum for a message object of type 'SetControllerConfiguration-request"
  "4158c0b1ce15d5824908b5157fccd994")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetControllerConfiguration-request>)))
  "Returns full string definition for message of type '<SetControllerConfiguration-request>"
  (cl:format cl:nil "ControllerConfiguration input~%~%================================================================================~%MSG: kortex_driver/ControllerConfiguration~%~%ControllerHandle handle~%string name~%MappingHandle active_mapping_handle~%string analog_input_identifier_enum~%string digital_input_identifier_enum~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetControllerConfiguration-request)))
  "Returns full string definition for message of type 'SetControllerConfiguration-request"
  (cl:format cl:nil "ControllerConfiguration input~%~%================================================================================~%MSG: kortex_driver/ControllerConfiguration~%~%ControllerHandle handle~%string name~%MappingHandle active_mapping_handle~%string analog_input_identifier_enum~%string digital_input_identifier_enum~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetControllerConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetControllerConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetControllerConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetControllerConfiguration-response.msg.html

(cl:defclass <SetControllerConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetControllerConfiguration-response (<SetControllerConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetControllerConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetControllerConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetControllerConfiguration-response> is deprecated: use kortex_driver-srv:SetControllerConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetControllerConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetControllerConfiguration-response>) ostream)
  "Serializes a message object of type '<SetControllerConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetControllerConfiguration-response>) istream)
  "Deserializes a message object of type '<SetControllerConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetControllerConfiguration-response>)))
  "Returns string type for a service object of type '<SetControllerConfiguration-response>"
  "kortex_driver/SetControllerConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControllerConfiguration-response)))
  "Returns string type for a service object of type 'SetControllerConfiguration-response"
  "kortex_driver/SetControllerConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetControllerConfiguration-response>)))
  "Returns md5sum for a message object of type '<SetControllerConfiguration-response>"
  "4158c0b1ce15d5824908b5157fccd994")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetControllerConfiguration-response)))
  "Returns md5sum for a message object of type 'SetControllerConfiguration-response"
  "4158c0b1ce15d5824908b5157fccd994")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetControllerConfiguration-response>)))
  "Returns full string definition for message of type '<SetControllerConfiguration-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetControllerConfiguration-response)))
  "Returns full string definition for message of type 'SetControllerConfiguration-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetControllerConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetControllerConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetControllerConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetControllerConfiguration)))
  'SetControllerConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetControllerConfiguration)))
  'SetControllerConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetControllerConfiguration)))
  "Returns string type for a service object of type '<SetControllerConfiguration>"
  "kortex_driver/SetControllerConfiguration")