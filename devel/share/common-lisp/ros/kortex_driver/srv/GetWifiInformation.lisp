; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetWifiInformation-request.msg.html

(cl:defclass <GetWifiInformation-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Ssid
    :initform (cl:make-instance 'kortex_driver-msg:Ssid)))
)

(cl:defclass GetWifiInformation-request (<GetWifiInformation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWifiInformation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWifiInformation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetWifiInformation-request> is deprecated: use kortex_driver-srv:GetWifiInformation-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetWifiInformation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWifiInformation-request>) ostream)
  "Serializes a message object of type '<GetWifiInformation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWifiInformation-request>) istream)
  "Deserializes a message object of type '<GetWifiInformation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWifiInformation-request>)))
  "Returns string type for a service object of type '<GetWifiInformation-request>"
  "kortex_driver/GetWifiInformationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWifiInformation-request)))
  "Returns string type for a service object of type 'GetWifiInformation-request"
  "kortex_driver/GetWifiInformationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWifiInformation-request>)))
  "Returns md5sum for a message object of type '<GetWifiInformation-request>"
  "54ddc4afd24b3f5a522f79c66970e8fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWifiInformation-request)))
  "Returns md5sum for a message object of type 'GetWifiInformation-request"
  "54ddc4afd24b3f5a522f79c66970e8fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWifiInformation-request>)))
  "Returns full string definition for message of type '<GetWifiInformation-request>"
  (cl:format cl:nil "Ssid input~%~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWifiInformation-request)))
  "Returns full string definition for message of type 'GetWifiInformation-request"
  (cl:format cl:nil "Ssid input~%~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWifiInformation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWifiInformation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWifiInformation-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetWifiInformation-response.msg.html

(cl:defclass <GetWifiInformation-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:WifiInformation
    :initform (cl:make-instance 'kortex_driver-msg:WifiInformation)))
)

(cl:defclass GetWifiInformation-response (<GetWifiInformation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWifiInformation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWifiInformation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetWifiInformation-response> is deprecated: use kortex_driver-srv:GetWifiInformation-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetWifiInformation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWifiInformation-response>) ostream)
  "Serializes a message object of type '<GetWifiInformation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWifiInformation-response>) istream)
  "Deserializes a message object of type '<GetWifiInformation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWifiInformation-response>)))
  "Returns string type for a service object of type '<GetWifiInformation-response>"
  "kortex_driver/GetWifiInformationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWifiInformation-response)))
  "Returns string type for a service object of type 'GetWifiInformation-response"
  "kortex_driver/GetWifiInformationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWifiInformation-response>)))
  "Returns md5sum for a message object of type '<GetWifiInformation-response>"
  "54ddc4afd24b3f5a522f79c66970e8fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWifiInformation-response)))
  "Returns md5sum for a message object of type 'GetWifiInformation-response"
  "54ddc4afd24b3f5a522f79c66970e8fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWifiInformation-response>)))
  "Returns full string definition for message of type '<GetWifiInformation-response>"
  (cl:format cl:nil "WifiInformation output~%~%================================================================================~%MSG: kortex_driver/WifiInformation~%~%Ssid ssid~%uint32 security_type~%uint32 encryption_type~%uint32 signal_quality~%int32 signal_strength~%uint32 frequency~%uint32 channel~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWifiInformation-response)))
  "Returns full string definition for message of type 'GetWifiInformation-response"
  (cl:format cl:nil "WifiInformation output~%~%================================================================================~%MSG: kortex_driver/WifiInformation~%~%Ssid ssid~%uint32 security_type~%uint32 encryption_type~%uint32 signal_quality~%int32 signal_strength~%uint32 frequency~%uint32 channel~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWifiInformation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWifiInformation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWifiInformation-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetWifiInformation)))
  'GetWifiInformation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetWifiInformation)))
  'GetWifiInformation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWifiInformation)))
  "Returns string type for a service object of type '<GetWifiInformation>"
  "kortex_driver/GetWifiInformation")