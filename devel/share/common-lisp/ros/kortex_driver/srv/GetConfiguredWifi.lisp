; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetConfiguredWifi-request.msg.html

(cl:defclass <GetConfiguredWifi-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Ssid
    :initform (cl:make-instance 'kortex_driver-msg:Ssid)))
)

(cl:defclass GetConfiguredWifi-request (<GetConfiguredWifi-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetConfiguredWifi-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetConfiguredWifi-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetConfiguredWifi-request> is deprecated: use kortex_driver-srv:GetConfiguredWifi-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetConfiguredWifi-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetConfiguredWifi-request>) ostream)
  "Serializes a message object of type '<GetConfiguredWifi-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetConfiguredWifi-request>) istream)
  "Deserializes a message object of type '<GetConfiguredWifi-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetConfiguredWifi-request>)))
  "Returns string type for a service object of type '<GetConfiguredWifi-request>"
  "kortex_driver/GetConfiguredWifiRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetConfiguredWifi-request)))
  "Returns string type for a service object of type 'GetConfiguredWifi-request"
  "kortex_driver/GetConfiguredWifiRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetConfiguredWifi-request>)))
  "Returns md5sum for a message object of type '<GetConfiguredWifi-request>"
  "5d1a4bbd14ba02b4b58bf8863a4fba51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetConfiguredWifi-request)))
  "Returns md5sum for a message object of type 'GetConfiguredWifi-request"
  "5d1a4bbd14ba02b4b58bf8863a4fba51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetConfiguredWifi-request>)))
  "Returns full string definition for message of type '<GetConfiguredWifi-request>"
  (cl:format cl:nil "Ssid input~%~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetConfiguredWifi-request)))
  "Returns full string definition for message of type 'GetConfiguredWifi-request"
  (cl:format cl:nil "Ssid input~%~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetConfiguredWifi-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetConfiguredWifi-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetConfiguredWifi-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetConfiguredWifi-response.msg.html

(cl:defclass <GetConfiguredWifi-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:WifiConfiguration
    :initform (cl:make-instance 'kortex_driver-msg:WifiConfiguration)))
)

(cl:defclass GetConfiguredWifi-response (<GetConfiguredWifi-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetConfiguredWifi-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetConfiguredWifi-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetConfiguredWifi-response> is deprecated: use kortex_driver-srv:GetConfiguredWifi-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetConfiguredWifi-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetConfiguredWifi-response>) ostream)
  "Serializes a message object of type '<GetConfiguredWifi-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetConfiguredWifi-response>) istream)
  "Deserializes a message object of type '<GetConfiguredWifi-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetConfiguredWifi-response>)))
  "Returns string type for a service object of type '<GetConfiguredWifi-response>"
  "kortex_driver/GetConfiguredWifiResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetConfiguredWifi-response)))
  "Returns string type for a service object of type 'GetConfiguredWifi-response"
  "kortex_driver/GetConfiguredWifiResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetConfiguredWifi-response>)))
  "Returns md5sum for a message object of type '<GetConfiguredWifi-response>"
  "5d1a4bbd14ba02b4b58bf8863a4fba51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetConfiguredWifi-response)))
  "Returns md5sum for a message object of type 'GetConfiguredWifi-response"
  "5d1a4bbd14ba02b4b58bf8863a4fba51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetConfiguredWifi-response>)))
  "Returns full string definition for message of type '<GetConfiguredWifi-response>"
  (cl:format cl:nil "WifiConfiguration output~%~%================================================================================~%MSG: kortex_driver/WifiConfiguration~%~%Ssid ssid~%string security_key~%bool connect_automatically~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetConfiguredWifi-response)))
  "Returns full string definition for message of type 'GetConfiguredWifi-response"
  (cl:format cl:nil "WifiConfiguration output~%~%================================================================================~%MSG: kortex_driver/WifiConfiguration~%~%Ssid ssid~%string security_key~%bool connect_automatically~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetConfiguredWifi-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetConfiguredWifi-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetConfiguredWifi-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetConfiguredWifi)))
  'GetConfiguredWifi-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetConfiguredWifi)))
  'GetConfiguredWifi-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetConfiguredWifi)))
  "Returns string type for a service object of type '<GetConfiguredWifi>"
  "kortex_driver/GetConfiguredWifi")