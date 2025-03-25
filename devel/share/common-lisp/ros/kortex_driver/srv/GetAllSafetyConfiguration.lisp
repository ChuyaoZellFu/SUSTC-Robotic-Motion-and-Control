; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetAllSafetyConfiguration-request.msg.html

(cl:defclass <GetAllSafetyConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetAllSafetyConfiguration-request (<GetAllSafetyConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllSafetyConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllSafetyConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllSafetyConfiguration-request> is deprecated: use kortex_driver-srv:GetAllSafetyConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetAllSafetyConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllSafetyConfiguration-request>) ostream)
  "Serializes a message object of type '<GetAllSafetyConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllSafetyConfiguration-request>) istream)
  "Deserializes a message object of type '<GetAllSafetyConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllSafetyConfiguration-request>)))
  "Returns string type for a service object of type '<GetAllSafetyConfiguration-request>"
  "kortex_driver/GetAllSafetyConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllSafetyConfiguration-request)))
  "Returns string type for a service object of type 'GetAllSafetyConfiguration-request"
  "kortex_driver/GetAllSafetyConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllSafetyConfiguration-request>)))
  "Returns md5sum for a message object of type '<GetAllSafetyConfiguration-request>"
  "332c0fb8c1c499e5ec5c674488e0110f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllSafetyConfiguration-request)))
  "Returns md5sum for a message object of type 'GetAllSafetyConfiguration-request"
  "332c0fb8c1c499e5ec5c674488e0110f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllSafetyConfiguration-request>)))
  "Returns full string definition for message of type '<GetAllSafetyConfiguration-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllSafetyConfiguration-request)))
  "Returns full string definition for message of type 'GetAllSafetyConfiguration-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllSafetyConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllSafetyConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllSafetyConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetAllSafetyConfiguration-response.msg.html

(cl:defclass <GetAllSafetyConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:SafetyConfigurationList
    :initform (cl:make-instance 'kortex_driver-msg:SafetyConfigurationList)))
)

(cl:defclass GetAllSafetyConfiguration-response (<GetAllSafetyConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllSafetyConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllSafetyConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllSafetyConfiguration-response> is deprecated: use kortex_driver-srv:GetAllSafetyConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetAllSafetyConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllSafetyConfiguration-response>) ostream)
  "Serializes a message object of type '<GetAllSafetyConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllSafetyConfiguration-response>) istream)
  "Deserializes a message object of type '<GetAllSafetyConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllSafetyConfiguration-response>)))
  "Returns string type for a service object of type '<GetAllSafetyConfiguration-response>"
  "kortex_driver/GetAllSafetyConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllSafetyConfiguration-response)))
  "Returns string type for a service object of type 'GetAllSafetyConfiguration-response"
  "kortex_driver/GetAllSafetyConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllSafetyConfiguration-response>)))
  "Returns md5sum for a message object of type '<GetAllSafetyConfiguration-response>"
  "332c0fb8c1c499e5ec5c674488e0110f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllSafetyConfiguration-response)))
  "Returns md5sum for a message object of type 'GetAllSafetyConfiguration-response"
  "332c0fb8c1c499e5ec5c674488e0110f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllSafetyConfiguration-response>)))
  "Returns full string definition for message of type '<GetAllSafetyConfiguration-response>"
  (cl:format cl:nil "SafetyConfigurationList output~%~%================================================================================~%MSG: kortex_driver/SafetyConfigurationList~%~%SafetyConfiguration[] configuration~%================================================================================~%MSG: kortex_driver/SafetyConfiguration~%~%SafetyHandle handle~%float32 error_threshold~%float32 warning_threshold~%SafetyEnable enable~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/SafetyEnable~%~%SafetyHandle handle~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllSafetyConfiguration-response)))
  "Returns full string definition for message of type 'GetAllSafetyConfiguration-response"
  (cl:format cl:nil "SafetyConfigurationList output~%~%================================================================================~%MSG: kortex_driver/SafetyConfigurationList~%~%SafetyConfiguration[] configuration~%================================================================================~%MSG: kortex_driver/SafetyConfiguration~%~%SafetyHandle handle~%float32 error_threshold~%float32 warning_threshold~%SafetyEnable enable~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/SafetyEnable~%~%SafetyHandle handle~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllSafetyConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllSafetyConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllSafetyConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAllSafetyConfiguration)))
  'GetAllSafetyConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAllSafetyConfiguration)))
  'GetAllSafetyConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllSafetyConfiguration)))
  "Returns string type for a service object of type '<GetAllSafetyConfiguration>"
  "kortex_driver/GetAllSafetyConfiguration")