; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetAvailableWifi-request.msg.html

(cl:defclass <GetAvailableWifi-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetAvailableWifi-request (<GetAvailableWifi-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAvailableWifi-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAvailableWifi-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAvailableWifi-request> is deprecated: use kortex_driver-srv:GetAvailableWifi-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetAvailableWifi-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAvailableWifi-request>) ostream)
  "Serializes a message object of type '<GetAvailableWifi-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAvailableWifi-request>) istream)
  "Deserializes a message object of type '<GetAvailableWifi-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAvailableWifi-request>)))
  "Returns string type for a service object of type '<GetAvailableWifi-request>"
  "kortex_driver/GetAvailableWifiRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAvailableWifi-request)))
  "Returns string type for a service object of type 'GetAvailableWifi-request"
  "kortex_driver/GetAvailableWifiRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAvailableWifi-request>)))
  "Returns md5sum for a message object of type '<GetAvailableWifi-request>"
  "7759436e458092e2d8f72d35d5d731f7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAvailableWifi-request)))
  "Returns md5sum for a message object of type 'GetAvailableWifi-request"
  "7759436e458092e2d8f72d35d5d731f7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAvailableWifi-request>)))
  "Returns full string definition for message of type '<GetAvailableWifi-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAvailableWifi-request)))
  "Returns full string definition for message of type 'GetAvailableWifi-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAvailableWifi-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAvailableWifi-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAvailableWifi-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetAvailableWifi-response.msg.html

(cl:defclass <GetAvailableWifi-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:WifiInformationList
    :initform (cl:make-instance 'kortex_driver-msg:WifiInformationList)))
)

(cl:defclass GetAvailableWifi-response (<GetAvailableWifi-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAvailableWifi-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAvailableWifi-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAvailableWifi-response> is deprecated: use kortex_driver-srv:GetAvailableWifi-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetAvailableWifi-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAvailableWifi-response>) ostream)
  "Serializes a message object of type '<GetAvailableWifi-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAvailableWifi-response>) istream)
  "Deserializes a message object of type '<GetAvailableWifi-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAvailableWifi-response>)))
  "Returns string type for a service object of type '<GetAvailableWifi-response>"
  "kortex_driver/GetAvailableWifiResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAvailableWifi-response)))
  "Returns string type for a service object of type 'GetAvailableWifi-response"
  "kortex_driver/GetAvailableWifiResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAvailableWifi-response>)))
  "Returns md5sum for a message object of type '<GetAvailableWifi-response>"
  "7759436e458092e2d8f72d35d5d731f7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAvailableWifi-response)))
  "Returns md5sum for a message object of type 'GetAvailableWifi-response"
  "7759436e458092e2d8f72d35d5d731f7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAvailableWifi-response>)))
  "Returns full string definition for message of type '<GetAvailableWifi-response>"
  (cl:format cl:nil "WifiInformationList output~%~%================================================================================~%MSG: kortex_driver/WifiInformationList~%~%WifiInformation[] wifi_information_list~%================================================================================~%MSG: kortex_driver/WifiInformation~%~%Ssid ssid~%uint32 security_type~%uint32 encryption_type~%uint32 signal_quality~%int32 signal_strength~%uint32 frequency~%uint32 channel~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAvailableWifi-response)))
  "Returns full string definition for message of type 'GetAvailableWifi-response"
  (cl:format cl:nil "WifiInformationList output~%~%================================================================================~%MSG: kortex_driver/WifiInformationList~%~%WifiInformation[] wifi_information_list~%================================================================================~%MSG: kortex_driver/WifiInformation~%~%Ssid ssid~%uint32 security_type~%uint32 encryption_type~%uint32 signal_quality~%int32 signal_strength~%uint32 frequency~%uint32 channel~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAvailableWifi-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAvailableWifi-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAvailableWifi-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAvailableWifi)))
  'GetAvailableWifi-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAvailableWifi)))
  'GetAvailableWifi-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAvailableWifi)))
  "Returns string type for a service object of type '<GetAvailableWifi>"
  "kortex_driver/GetAvailableWifi")