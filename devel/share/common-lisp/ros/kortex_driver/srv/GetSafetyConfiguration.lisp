; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetSafetyConfiguration-request.msg.html

(cl:defclass <GetSafetyConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SafetyHandle
    :initform (cl:make-instance 'kortex_driver-msg:SafetyHandle)))
)

(cl:defclass GetSafetyConfiguration-request (<GetSafetyConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSafetyConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSafetyConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetSafetyConfiguration-request> is deprecated: use kortex_driver-srv:GetSafetyConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetSafetyConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSafetyConfiguration-request>) ostream)
  "Serializes a message object of type '<GetSafetyConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSafetyConfiguration-request>) istream)
  "Deserializes a message object of type '<GetSafetyConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSafetyConfiguration-request>)))
  "Returns string type for a service object of type '<GetSafetyConfiguration-request>"
  "kortex_driver/GetSafetyConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSafetyConfiguration-request)))
  "Returns string type for a service object of type 'GetSafetyConfiguration-request"
  "kortex_driver/GetSafetyConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSafetyConfiguration-request>)))
  "Returns md5sum for a message object of type '<GetSafetyConfiguration-request>"
  "943b22eac92db7c1fc8c0b5bbdfb1565")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSafetyConfiguration-request)))
  "Returns md5sum for a message object of type 'GetSafetyConfiguration-request"
  "943b22eac92db7c1fc8c0b5bbdfb1565")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSafetyConfiguration-request>)))
  "Returns full string definition for message of type '<GetSafetyConfiguration-request>"
  (cl:format cl:nil "SafetyHandle input~%~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSafetyConfiguration-request)))
  "Returns full string definition for message of type 'GetSafetyConfiguration-request"
  (cl:format cl:nil "SafetyHandle input~%~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSafetyConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSafetyConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSafetyConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetSafetyConfiguration-response.msg.html

(cl:defclass <GetSafetyConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:SafetyConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:SafetyConfiguration)))
)

(cl:defclass GetSafetyConfiguration-response (<GetSafetyConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSafetyConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSafetyConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetSafetyConfiguration-response> is deprecated: use kortex_driver-srv:GetSafetyConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetSafetyConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSafetyConfiguration-response>) ostream)
  "Serializes a message object of type '<GetSafetyConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSafetyConfiguration-response>) istream)
  "Deserializes a message object of type '<GetSafetyConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSafetyConfiguration-response>)))
  "Returns string type for a service object of type '<GetSafetyConfiguration-response>"
  "kortex_driver/GetSafetyConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSafetyConfiguration-response)))
  "Returns string type for a service object of type 'GetSafetyConfiguration-response"
  "kortex_driver/GetSafetyConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSafetyConfiguration-response>)))
  "Returns md5sum for a message object of type '<GetSafetyConfiguration-response>"
  "943b22eac92db7c1fc8c0b5bbdfb1565")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSafetyConfiguration-response)))
  "Returns md5sum for a message object of type 'GetSafetyConfiguration-response"
  "943b22eac92db7c1fc8c0b5bbdfb1565")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSafetyConfiguration-response>)))
  "Returns full string definition for message of type '<GetSafetyConfiguration-response>"
  (cl:format cl:nil "SafetyConfiguration output~%~%================================================================================~%MSG: kortex_driver/SafetyConfiguration~%~%SafetyHandle handle~%float32 error_threshold~%float32 warning_threshold~%SafetyEnable enable~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/SafetyEnable~%~%SafetyHandle handle~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSafetyConfiguration-response)))
  "Returns full string definition for message of type 'GetSafetyConfiguration-response"
  (cl:format cl:nil "SafetyConfiguration output~%~%================================================================================~%MSG: kortex_driver/SafetyConfiguration~%~%SafetyHandle handle~%float32 error_threshold~%float32 warning_threshold~%SafetyEnable enable~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/SafetyEnable~%~%SafetyHandle handle~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSafetyConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSafetyConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSafetyConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetSafetyConfiguration)))
  'GetSafetyConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetSafetyConfiguration)))
  'GetSafetyConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSafetyConfiguration)))
  "Returns string type for a service object of type '<GetSafetyConfiguration>"
  "kortex_driver/GetSafetyConfiguration")