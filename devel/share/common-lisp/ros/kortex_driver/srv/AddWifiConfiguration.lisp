; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude AddWifiConfiguration-request.msg.html

(cl:defclass <AddWifiConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:WifiConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:WifiConfiguration)))
)

(cl:defclass AddWifiConfiguration-request (<AddWifiConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddWifiConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddWifiConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<AddWifiConfiguration-request> is deprecated: use kortex_driver-srv:AddWifiConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <AddWifiConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddWifiConfiguration-request>) ostream)
  "Serializes a message object of type '<AddWifiConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddWifiConfiguration-request>) istream)
  "Deserializes a message object of type '<AddWifiConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddWifiConfiguration-request>)))
  "Returns string type for a service object of type '<AddWifiConfiguration-request>"
  "kortex_driver/AddWifiConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddWifiConfiguration-request)))
  "Returns string type for a service object of type 'AddWifiConfiguration-request"
  "kortex_driver/AddWifiConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddWifiConfiguration-request>)))
  "Returns md5sum for a message object of type '<AddWifiConfiguration-request>"
  "2cc46423b77629bdba4927dbabe3a2cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddWifiConfiguration-request)))
  "Returns md5sum for a message object of type 'AddWifiConfiguration-request"
  "2cc46423b77629bdba4927dbabe3a2cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddWifiConfiguration-request>)))
  "Returns full string definition for message of type '<AddWifiConfiguration-request>"
  (cl:format cl:nil "WifiConfiguration input~%~%================================================================================~%MSG: kortex_driver/WifiConfiguration~%~%Ssid ssid~%string security_key~%bool connect_automatically~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddWifiConfiguration-request)))
  "Returns full string definition for message of type 'AddWifiConfiguration-request"
  (cl:format cl:nil "WifiConfiguration input~%~%================================================================================~%MSG: kortex_driver/WifiConfiguration~%~%Ssid ssid~%string security_key~%bool connect_automatically~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddWifiConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddWifiConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AddWifiConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude AddWifiConfiguration-response.msg.html

(cl:defclass <AddWifiConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass AddWifiConfiguration-response (<AddWifiConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddWifiConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddWifiConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<AddWifiConfiguration-response> is deprecated: use kortex_driver-srv:AddWifiConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <AddWifiConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddWifiConfiguration-response>) ostream)
  "Serializes a message object of type '<AddWifiConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddWifiConfiguration-response>) istream)
  "Deserializes a message object of type '<AddWifiConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddWifiConfiguration-response>)))
  "Returns string type for a service object of type '<AddWifiConfiguration-response>"
  "kortex_driver/AddWifiConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddWifiConfiguration-response)))
  "Returns string type for a service object of type 'AddWifiConfiguration-response"
  "kortex_driver/AddWifiConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddWifiConfiguration-response>)))
  "Returns md5sum for a message object of type '<AddWifiConfiguration-response>"
  "2cc46423b77629bdba4927dbabe3a2cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddWifiConfiguration-response)))
  "Returns md5sum for a message object of type 'AddWifiConfiguration-response"
  "2cc46423b77629bdba4927dbabe3a2cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddWifiConfiguration-response>)))
  "Returns full string definition for message of type '<AddWifiConfiguration-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddWifiConfiguration-response)))
  "Returns full string definition for message of type 'AddWifiConfiguration-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddWifiConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddWifiConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AddWifiConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AddWifiConfiguration)))
  'AddWifiConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AddWifiConfiguration)))
  'AddWifiConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddWifiConfiguration)))
  "Returns string type for a service object of type '<AddWifiConfiguration>"
  "kortex_driver/AddWifiConfiguration")