; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetAllSafetyInformation-request.msg.html

(cl:defclass <GetAllSafetyInformation-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetAllSafetyInformation-request (<GetAllSafetyInformation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllSafetyInformation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllSafetyInformation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllSafetyInformation-request> is deprecated: use kortex_driver-srv:GetAllSafetyInformation-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetAllSafetyInformation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllSafetyInformation-request>) ostream)
  "Serializes a message object of type '<GetAllSafetyInformation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllSafetyInformation-request>) istream)
  "Deserializes a message object of type '<GetAllSafetyInformation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllSafetyInformation-request>)))
  "Returns string type for a service object of type '<GetAllSafetyInformation-request>"
  "kortex_driver/GetAllSafetyInformationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllSafetyInformation-request)))
  "Returns string type for a service object of type 'GetAllSafetyInformation-request"
  "kortex_driver/GetAllSafetyInformationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllSafetyInformation-request>)))
  "Returns md5sum for a message object of type '<GetAllSafetyInformation-request>"
  "6cd621443d851423fb32151d65f5f576")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllSafetyInformation-request)))
  "Returns md5sum for a message object of type 'GetAllSafetyInformation-request"
  "6cd621443d851423fb32151d65f5f576")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllSafetyInformation-request>)))
  "Returns full string definition for message of type '<GetAllSafetyInformation-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllSafetyInformation-request)))
  "Returns full string definition for message of type 'GetAllSafetyInformation-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllSafetyInformation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllSafetyInformation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllSafetyInformation-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetAllSafetyInformation-response.msg.html

(cl:defclass <GetAllSafetyInformation-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:SafetyInformationList
    :initform (cl:make-instance 'kortex_driver-msg:SafetyInformationList)))
)

(cl:defclass GetAllSafetyInformation-response (<GetAllSafetyInformation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllSafetyInformation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllSafetyInformation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllSafetyInformation-response> is deprecated: use kortex_driver-srv:GetAllSafetyInformation-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetAllSafetyInformation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllSafetyInformation-response>) ostream)
  "Serializes a message object of type '<GetAllSafetyInformation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllSafetyInformation-response>) istream)
  "Deserializes a message object of type '<GetAllSafetyInformation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllSafetyInformation-response>)))
  "Returns string type for a service object of type '<GetAllSafetyInformation-response>"
  "kortex_driver/GetAllSafetyInformationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllSafetyInformation-response)))
  "Returns string type for a service object of type 'GetAllSafetyInformation-response"
  "kortex_driver/GetAllSafetyInformationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllSafetyInformation-response>)))
  "Returns md5sum for a message object of type '<GetAllSafetyInformation-response>"
  "6cd621443d851423fb32151d65f5f576")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllSafetyInformation-response)))
  "Returns md5sum for a message object of type 'GetAllSafetyInformation-response"
  "6cd621443d851423fb32151d65f5f576")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllSafetyInformation-response>)))
  "Returns full string definition for message of type '<GetAllSafetyInformation-response>"
  (cl:format cl:nil "SafetyInformationList output~%~%================================================================================~%MSG: kortex_driver/SafetyInformationList~%~%SafetyInformation[] information~%================================================================================~%MSG: kortex_driver/SafetyInformation~%~%SafetyHandle handle~%bool can_change_safety_state~%bool has_warning_threshold~%bool has_error_threshold~%uint32 limit_type~%float32 default_warning_threshold~%float32 default_error_threshold~%float32 upper_hard_limit~%float32 lower_hard_limit~%uint32 status~%uint32 unit~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllSafetyInformation-response)))
  "Returns full string definition for message of type 'GetAllSafetyInformation-response"
  (cl:format cl:nil "SafetyInformationList output~%~%================================================================================~%MSG: kortex_driver/SafetyInformationList~%~%SafetyInformation[] information~%================================================================================~%MSG: kortex_driver/SafetyInformation~%~%SafetyHandle handle~%bool can_change_safety_state~%bool has_warning_threshold~%bool has_error_threshold~%uint32 limit_type~%float32 default_warning_threshold~%float32 default_error_threshold~%float32 upper_hard_limit~%float32 lower_hard_limit~%uint32 status~%uint32 unit~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllSafetyInformation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllSafetyInformation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllSafetyInformation-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAllSafetyInformation)))
  'GetAllSafetyInformation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAllSafetyInformation)))
  'GetAllSafetyInformation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllSafetyInformation)))
  "Returns string type for a service object of type '<GetAllSafetyInformation>"
  "kortex_driver/GetAllSafetyInformation")