; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetWifiCountryCode-request.msg.html

(cl:defclass <GetWifiCountryCode-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetWifiCountryCode-request (<GetWifiCountryCode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWifiCountryCode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWifiCountryCode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetWifiCountryCode-request> is deprecated: use kortex_driver-srv:GetWifiCountryCode-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetWifiCountryCode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWifiCountryCode-request>) ostream)
  "Serializes a message object of type '<GetWifiCountryCode-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWifiCountryCode-request>) istream)
  "Deserializes a message object of type '<GetWifiCountryCode-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWifiCountryCode-request>)))
  "Returns string type for a service object of type '<GetWifiCountryCode-request>"
  "kortex_driver/GetWifiCountryCodeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWifiCountryCode-request)))
  "Returns string type for a service object of type 'GetWifiCountryCode-request"
  "kortex_driver/GetWifiCountryCodeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWifiCountryCode-request>)))
  "Returns md5sum for a message object of type '<GetWifiCountryCode-request>"
  "e29a445c35ddd6d1bc1a1daa766b9db1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWifiCountryCode-request)))
  "Returns md5sum for a message object of type 'GetWifiCountryCode-request"
  "e29a445c35ddd6d1bc1a1daa766b9db1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWifiCountryCode-request>)))
  "Returns full string definition for message of type '<GetWifiCountryCode-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWifiCountryCode-request)))
  "Returns full string definition for message of type 'GetWifiCountryCode-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWifiCountryCode-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWifiCountryCode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWifiCountryCode-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetWifiCountryCode-response.msg.html

(cl:defclass <GetWifiCountryCode-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:CountryCode
    :initform (cl:make-instance 'kortex_driver-msg:CountryCode)))
)

(cl:defclass GetWifiCountryCode-response (<GetWifiCountryCode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWifiCountryCode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWifiCountryCode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetWifiCountryCode-response> is deprecated: use kortex_driver-srv:GetWifiCountryCode-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetWifiCountryCode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWifiCountryCode-response>) ostream)
  "Serializes a message object of type '<GetWifiCountryCode-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWifiCountryCode-response>) istream)
  "Deserializes a message object of type '<GetWifiCountryCode-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWifiCountryCode-response>)))
  "Returns string type for a service object of type '<GetWifiCountryCode-response>"
  "kortex_driver/GetWifiCountryCodeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWifiCountryCode-response)))
  "Returns string type for a service object of type 'GetWifiCountryCode-response"
  "kortex_driver/GetWifiCountryCodeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWifiCountryCode-response>)))
  "Returns md5sum for a message object of type '<GetWifiCountryCode-response>"
  "e29a445c35ddd6d1bc1a1daa766b9db1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWifiCountryCode-response)))
  "Returns md5sum for a message object of type 'GetWifiCountryCode-response"
  "e29a445c35ddd6d1bc1a1daa766b9db1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWifiCountryCode-response>)))
  "Returns full string definition for message of type '<GetWifiCountryCode-response>"
  (cl:format cl:nil "CountryCode output~%~%================================================================================~%MSG: kortex_driver/CountryCode~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWifiCountryCode-response)))
  "Returns full string definition for message of type 'GetWifiCountryCode-response"
  (cl:format cl:nil "CountryCode output~%~%================================================================================~%MSG: kortex_driver/CountryCode~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWifiCountryCode-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWifiCountryCode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWifiCountryCode-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetWifiCountryCode)))
  'GetWifiCountryCode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetWifiCountryCode)))
  'GetWifiCountryCode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWifiCountryCode)))
  "Returns string type for a service object of type '<GetWifiCountryCode>"
  "kortex_driver/GetWifiCountryCode")