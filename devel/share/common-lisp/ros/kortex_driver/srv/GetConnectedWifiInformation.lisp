; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetConnectedWifiInformation-request.msg.html

(cl:defclass <GetConnectedWifiInformation-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetConnectedWifiInformation-request (<GetConnectedWifiInformation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetConnectedWifiInformation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetConnectedWifiInformation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetConnectedWifiInformation-request> is deprecated: use kortex_driver-srv:GetConnectedWifiInformation-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetConnectedWifiInformation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetConnectedWifiInformation-request>) ostream)
  "Serializes a message object of type '<GetConnectedWifiInformation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetConnectedWifiInformation-request>) istream)
  "Deserializes a message object of type '<GetConnectedWifiInformation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetConnectedWifiInformation-request>)))
  "Returns string type for a service object of type '<GetConnectedWifiInformation-request>"
  "kortex_driver/GetConnectedWifiInformationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetConnectedWifiInformation-request)))
  "Returns string type for a service object of type 'GetConnectedWifiInformation-request"
  "kortex_driver/GetConnectedWifiInformationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetConnectedWifiInformation-request>)))
  "Returns md5sum for a message object of type '<GetConnectedWifiInformation-request>"
  "38ff400e5249598bc1f7289a3e9733aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetConnectedWifiInformation-request)))
  "Returns md5sum for a message object of type 'GetConnectedWifiInformation-request"
  "38ff400e5249598bc1f7289a3e9733aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetConnectedWifiInformation-request>)))
  "Returns full string definition for message of type '<GetConnectedWifiInformation-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetConnectedWifiInformation-request)))
  "Returns full string definition for message of type 'GetConnectedWifiInformation-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetConnectedWifiInformation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetConnectedWifiInformation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetConnectedWifiInformation-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetConnectedWifiInformation-response.msg.html

(cl:defclass <GetConnectedWifiInformation-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:WifiInformation
    :initform (cl:make-instance 'kortex_driver-msg:WifiInformation)))
)

(cl:defclass GetConnectedWifiInformation-response (<GetConnectedWifiInformation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetConnectedWifiInformation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetConnectedWifiInformation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetConnectedWifiInformation-response> is deprecated: use kortex_driver-srv:GetConnectedWifiInformation-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetConnectedWifiInformation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetConnectedWifiInformation-response>) ostream)
  "Serializes a message object of type '<GetConnectedWifiInformation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetConnectedWifiInformation-response>) istream)
  "Deserializes a message object of type '<GetConnectedWifiInformation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetConnectedWifiInformation-response>)))
  "Returns string type for a service object of type '<GetConnectedWifiInformation-response>"
  "kortex_driver/GetConnectedWifiInformationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetConnectedWifiInformation-response)))
  "Returns string type for a service object of type 'GetConnectedWifiInformation-response"
  "kortex_driver/GetConnectedWifiInformationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetConnectedWifiInformation-response>)))
  "Returns md5sum for a message object of type '<GetConnectedWifiInformation-response>"
  "38ff400e5249598bc1f7289a3e9733aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetConnectedWifiInformation-response)))
  "Returns md5sum for a message object of type 'GetConnectedWifiInformation-response"
  "38ff400e5249598bc1f7289a3e9733aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetConnectedWifiInformation-response>)))
  "Returns full string definition for message of type '<GetConnectedWifiInformation-response>"
  (cl:format cl:nil "WifiInformation output~%~%================================================================================~%MSG: kortex_driver/WifiInformation~%~%Ssid ssid~%uint32 security_type~%uint32 encryption_type~%uint32 signal_quality~%int32 signal_strength~%uint32 frequency~%uint32 channel~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetConnectedWifiInformation-response)))
  "Returns full string definition for message of type 'GetConnectedWifiInformation-response"
  (cl:format cl:nil "WifiInformation output~%~%================================================================================~%MSG: kortex_driver/WifiInformation~%~%Ssid ssid~%uint32 security_type~%uint32 encryption_type~%uint32 signal_quality~%int32 signal_strength~%uint32 frequency~%uint32 channel~%================================================================================~%MSG: kortex_driver/Ssid~%~%string identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetConnectedWifiInformation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetConnectedWifiInformation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetConnectedWifiInformation-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetConnectedWifiInformation)))
  'GetConnectedWifiInformation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetConnectedWifiInformation)))
  'GetConnectedWifiInformation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetConnectedWifiInformation)))
  "Returns string type for a service object of type '<GetConnectedWifiInformation>"
  "kortex_driver/GetConnectedWifiInformation")