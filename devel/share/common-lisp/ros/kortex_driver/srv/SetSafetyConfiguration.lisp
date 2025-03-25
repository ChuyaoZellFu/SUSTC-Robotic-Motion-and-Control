; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetSafetyConfiguration-request.msg.html

(cl:defclass <SetSafetyConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:SafetyConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:SafetyConfiguration)))
)

(cl:defclass SetSafetyConfiguration-request (<SetSafetyConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSafetyConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSafetyConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetSafetyConfiguration-request> is deprecated: use kortex_driver-srv:SetSafetyConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetSafetyConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSafetyConfiguration-request>) ostream)
  "Serializes a message object of type '<SetSafetyConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSafetyConfiguration-request>) istream)
  "Deserializes a message object of type '<SetSafetyConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSafetyConfiguration-request>)))
  "Returns string type for a service object of type '<SetSafetyConfiguration-request>"
  "kortex_driver/SetSafetyConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSafetyConfiguration-request)))
  "Returns string type for a service object of type 'SetSafetyConfiguration-request"
  "kortex_driver/SetSafetyConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSafetyConfiguration-request>)))
  "Returns md5sum for a message object of type '<SetSafetyConfiguration-request>"
  "b1a9f41cb42ab39cba6bfb32d20fea4f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSafetyConfiguration-request)))
  "Returns md5sum for a message object of type 'SetSafetyConfiguration-request"
  "b1a9f41cb42ab39cba6bfb32d20fea4f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSafetyConfiguration-request>)))
  "Returns full string definition for message of type '<SetSafetyConfiguration-request>"
  (cl:format cl:nil "SafetyConfiguration input~%~%================================================================================~%MSG: kortex_driver/SafetyConfiguration~%~%SafetyHandle handle~%float32 error_threshold~%float32 warning_threshold~%SafetyEnable enable~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/SafetyEnable~%~%SafetyHandle handle~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSafetyConfiguration-request)))
  "Returns full string definition for message of type 'SetSafetyConfiguration-request"
  (cl:format cl:nil "SafetyConfiguration input~%~%================================================================================~%MSG: kortex_driver/SafetyConfiguration~%~%SafetyHandle handle~%float32 error_threshold~%float32 warning_threshold~%SafetyEnable enable~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/SafetyEnable~%~%SafetyHandle handle~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSafetyConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSafetyConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSafetyConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetSafetyConfiguration-response.msg.html

(cl:defclass <SetSafetyConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetSafetyConfiguration-response (<SetSafetyConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSafetyConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSafetyConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetSafetyConfiguration-response> is deprecated: use kortex_driver-srv:SetSafetyConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetSafetyConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSafetyConfiguration-response>) ostream)
  "Serializes a message object of type '<SetSafetyConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSafetyConfiguration-response>) istream)
  "Deserializes a message object of type '<SetSafetyConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSafetyConfiguration-response>)))
  "Returns string type for a service object of type '<SetSafetyConfiguration-response>"
  "kortex_driver/SetSafetyConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSafetyConfiguration-response)))
  "Returns string type for a service object of type 'SetSafetyConfiguration-response"
  "kortex_driver/SetSafetyConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSafetyConfiguration-response>)))
  "Returns md5sum for a message object of type '<SetSafetyConfiguration-response>"
  "b1a9f41cb42ab39cba6bfb32d20fea4f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSafetyConfiguration-response)))
  "Returns md5sum for a message object of type 'SetSafetyConfiguration-response"
  "b1a9f41cb42ab39cba6bfb32d20fea4f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSafetyConfiguration-response>)))
  "Returns full string definition for message of type '<SetSafetyConfiguration-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSafetyConfiguration-response)))
  "Returns full string definition for message of type 'SetSafetyConfiguration-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSafetyConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSafetyConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSafetyConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetSafetyConfiguration)))
  'SetSafetyConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetSafetyConfiguration)))
  'SetSafetyConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSafetyConfiguration)))
  "Returns string type for a service object of type '<SetSafetyConfiguration>"
  "kortex_driver/SetSafetyConfiguration")