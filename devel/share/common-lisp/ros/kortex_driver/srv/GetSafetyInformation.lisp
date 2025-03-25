; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetSafetyInformation-request.msg.html

(cl:defclass <GetSafetyInformation-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SafetyHandle
    :initform (cl:make-instance 'kortex_driver-msg:SafetyHandle)))
)

(cl:defclass GetSafetyInformation-request (<GetSafetyInformation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSafetyInformation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSafetyInformation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetSafetyInformation-request> is deprecated: use kortex_driver-srv:GetSafetyInformation-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetSafetyInformation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSafetyInformation-request>) ostream)
  "Serializes a message object of type '<GetSafetyInformation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSafetyInformation-request>) istream)
  "Deserializes a message object of type '<GetSafetyInformation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSafetyInformation-request>)))
  "Returns string type for a service object of type '<GetSafetyInformation-request>"
  "kortex_driver/GetSafetyInformationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSafetyInformation-request)))
  "Returns string type for a service object of type 'GetSafetyInformation-request"
  "kortex_driver/GetSafetyInformationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSafetyInformation-request>)))
  "Returns md5sum for a message object of type '<GetSafetyInformation-request>"
  "03aeca1f2d2314fba73be4aeb3606ece")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSafetyInformation-request)))
  "Returns md5sum for a message object of type 'GetSafetyInformation-request"
  "03aeca1f2d2314fba73be4aeb3606ece")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSafetyInformation-request>)))
  "Returns full string definition for message of type '<GetSafetyInformation-request>"
  (cl:format cl:nil "SafetyHandle input~%~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSafetyInformation-request)))
  "Returns full string definition for message of type 'GetSafetyInformation-request"
  (cl:format cl:nil "SafetyHandle input~%~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSafetyInformation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSafetyInformation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSafetyInformation-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetSafetyInformation-response.msg.html

(cl:defclass <GetSafetyInformation-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:SafetyInformation
    :initform (cl:make-instance 'kortex_driver-msg:SafetyInformation)))
)

(cl:defclass GetSafetyInformation-response (<GetSafetyInformation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSafetyInformation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSafetyInformation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetSafetyInformation-response> is deprecated: use kortex_driver-srv:GetSafetyInformation-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetSafetyInformation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSafetyInformation-response>) ostream)
  "Serializes a message object of type '<GetSafetyInformation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSafetyInformation-response>) istream)
  "Deserializes a message object of type '<GetSafetyInformation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSafetyInformation-response>)))
  "Returns string type for a service object of type '<GetSafetyInformation-response>"
  "kortex_driver/GetSafetyInformationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSafetyInformation-response)))
  "Returns string type for a service object of type 'GetSafetyInformation-response"
  "kortex_driver/GetSafetyInformationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSafetyInformation-response>)))
  "Returns md5sum for a message object of type '<GetSafetyInformation-response>"
  "03aeca1f2d2314fba73be4aeb3606ece")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSafetyInformation-response)))
  "Returns md5sum for a message object of type 'GetSafetyInformation-response"
  "03aeca1f2d2314fba73be4aeb3606ece")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSafetyInformation-response>)))
  "Returns full string definition for message of type '<GetSafetyInformation-response>"
  (cl:format cl:nil "SafetyInformation output~%~%================================================================================~%MSG: kortex_driver/SafetyInformation~%~%SafetyHandle handle~%bool can_change_safety_state~%bool has_warning_threshold~%bool has_error_threshold~%uint32 limit_type~%float32 default_warning_threshold~%float32 default_error_threshold~%float32 upper_hard_limit~%float32 lower_hard_limit~%uint32 status~%uint32 unit~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSafetyInformation-response)))
  "Returns full string definition for message of type 'GetSafetyInformation-response"
  (cl:format cl:nil "SafetyInformation output~%~%================================================================================~%MSG: kortex_driver/SafetyInformation~%~%SafetyHandle handle~%bool can_change_safety_state~%bool has_warning_threshold~%bool has_error_threshold~%uint32 limit_type~%float32 default_warning_threshold~%float32 default_error_threshold~%float32 upper_hard_limit~%float32 lower_hard_limit~%uint32 status~%uint32 unit~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSafetyInformation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSafetyInformation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSafetyInformation-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetSafetyInformation)))
  'GetSafetyInformation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetSafetyInformation)))
  'GetSafetyInformation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSafetyInformation)))
  "Returns string type for a service object of type '<GetSafetyInformation>"
  "kortex_driver/GetSafetyInformation")