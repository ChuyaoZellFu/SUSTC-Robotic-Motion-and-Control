; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetAllConfiguredWifis-request.msg.html

(cl:defclass <GetAllConfiguredWifis-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetAllConfiguredWifis-request (<GetAllConfiguredWifis-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllConfiguredWifis-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllConfiguredWifis-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllConfiguredWifis-request> is deprecated: use kortex_driver-srv:GetAllConfiguredWifis-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetAllConfiguredWifis-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllConfiguredWifis-request>) ostream)
  "Serializes a message object of type '<GetAllConfiguredWifis-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllConfiguredWifis-request>) istream)
  "Deserializes a message object of type '<GetAllConfiguredWifis-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllConfiguredWifis-request>)))
  "Returns string type for a service object of type '<GetAllConfiguredWifis-request>"
  "kortex_driver/GetAllConfiguredWifisRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllConfiguredWifis-request)))
  "Returns string type for a service object of type 'GetAllConfiguredWifis-request"
  "kortex_driver/GetAllConfiguredWifisRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllConfiguredWifis-request>)))
  "Returns md5sum for a message object of type '<GetAllConfiguredWifis-request>"
  "00cc4429e5b474aecc287723cc4eff69")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllConfiguredWifis-request)))
  "Returns md5sum for a message object of type 'GetAllConfiguredWifis-request"
  "00cc4429e5b474aecc287723cc4eff69")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllConfiguredWifis-request>)))
  "Returns full string definition for message of type '<GetAllConfiguredWifis-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllConfiguredWifis-request)))
  "Returns full string definition for message of type 'GetAllConfiguredWifis-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllConfiguredWifis-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllConfiguredWifis-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllConfiguredWifis-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetAllConfiguredWifis-response.msg.html

(cl:defclass <GetAllConfiguredWifis-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:WifiConfigurationList
    :initform (cl:make-instance 'kortex_driver-msg:WifiConfigurationList)))
)

(cl:defclass GetAllConfiguredWifis-response (<GetAllConfiguredWifis-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllConfiguredWifis-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllConfiguredWifis-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAllConfiguredWifis-response> is deprecated: use kortex_driver-srv:GetAllConfiguredWifis-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetAllConfiguredWifis-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllConfiguredWifis-response>) ostream)
  "Serializes a message object of type '<GetAllConfiguredWifis-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllConfiguredWifis-response>) istream)
  "Deserializes a message object of type '<GetAllConfiguredWifis-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllConfiguredWifis-response>)))
  "Returns string type for a service object of type '<GetAllConfiguredWifis-response>"
  "kortex_driver/GetAllConfiguredWifisResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllConfiguredWifis-response)))
  "Returns string type for a service object of type 'GetAllConfiguredWifis-response"
  "kortex_driver/GetAllConfiguredWifisResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllConfiguredWifis-response>)))
  "Returns md5sum for a message object of type '<GetAllConfiguredWifis-response>"
  "00cc4429e5b474aecc287723cc4eff69")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllConfiguredWifis-response)))
  "Returns md5sum for a message object of type 'GetAllConfiguredWifis-response"
  "00cc4429e5b474aecc287723cc4eff69")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllConfiguredWifis-response>)))
  "Returns full string definition for message of type '<GetAllConfiguredWifis-response>"
  (cl:format cl:nil "WifiConfigurationList output~%~%================================================================================~%MSG: kortex_driver/WifiConfigurationList~%~%WifiConfiguration[] wifi_configuration_list~%================================================================================~%MSG: kortex_driver/WifiConfiguration~%~%Ssid ssid~%string security_key~%bool connect_automatically~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllConfiguredWifis-response)))
  "Returns full string definition for message of type 'GetAllConfiguredWifis-response"
  (cl:format cl:nil "WifiConfigurationList output~%~%================================================================================~%MSG: kortex_driver/WifiConfigurationList~%~%WifiConfiguration[] wifi_configuration_list~%================================================================================~%MSG: kortex_driver/WifiConfiguration~%~%Ssid ssid~%string security_key~%bool connect_automatically~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllConfiguredWifis-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllConfiguredWifis-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllConfiguredWifis-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAllConfiguredWifis)))
  'GetAllConfiguredWifis-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAllConfiguredWifis)))
  'GetAllConfiguredWifis-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllConfiguredWifis)))
  "Returns string type for a service object of type '<GetAllConfiguredWifis>"
  "kortex_driver/GetAllConfiguredWifis")