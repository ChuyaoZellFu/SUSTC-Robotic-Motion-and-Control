; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetWifiCountryCode-request.msg.html

(cl:defclass <SetWifiCountryCode-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:CountryCode
    :initform (cl:make-instance 'kortex_driver-msg:CountryCode)))
)

(cl:defclass SetWifiCountryCode-request (<SetWifiCountryCode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetWifiCountryCode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetWifiCountryCode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetWifiCountryCode-request> is deprecated: use kortex_driver-srv:SetWifiCountryCode-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetWifiCountryCode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetWifiCountryCode-request>) ostream)
  "Serializes a message object of type '<SetWifiCountryCode-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetWifiCountryCode-request>) istream)
  "Deserializes a message object of type '<SetWifiCountryCode-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetWifiCountryCode-request>)))
  "Returns string type for a service object of type '<SetWifiCountryCode-request>"
  "kortex_driver/SetWifiCountryCodeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetWifiCountryCode-request)))
  "Returns string type for a service object of type 'SetWifiCountryCode-request"
  "kortex_driver/SetWifiCountryCodeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetWifiCountryCode-request>)))
  "Returns md5sum for a message object of type '<SetWifiCountryCode-request>"
  "7960dab80443332660b3bb4bf774c9ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetWifiCountryCode-request)))
  "Returns md5sum for a message object of type 'SetWifiCountryCode-request"
  "7960dab80443332660b3bb4bf774c9ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetWifiCountryCode-request>)))
  "Returns full string definition for message of type '<SetWifiCountryCode-request>"
  (cl:format cl:nil "CountryCode input~%~%================================================================================~%MSG: kortex_driver/CountryCode~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetWifiCountryCode-request)))
  "Returns full string definition for message of type 'SetWifiCountryCode-request"
  (cl:format cl:nil "CountryCode input~%~%================================================================================~%MSG: kortex_driver/CountryCode~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetWifiCountryCode-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetWifiCountryCode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetWifiCountryCode-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetWifiCountryCode-response.msg.html

(cl:defclass <SetWifiCountryCode-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetWifiCountryCode-response (<SetWifiCountryCode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetWifiCountryCode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetWifiCountryCode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetWifiCountryCode-response> is deprecated: use kortex_driver-srv:SetWifiCountryCode-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetWifiCountryCode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetWifiCountryCode-response>) ostream)
  "Serializes a message object of type '<SetWifiCountryCode-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetWifiCountryCode-response>) istream)
  "Deserializes a message object of type '<SetWifiCountryCode-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetWifiCountryCode-response>)))
  "Returns string type for a service object of type '<SetWifiCountryCode-response>"
  "kortex_driver/SetWifiCountryCodeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetWifiCountryCode-response)))
  "Returns string type for a service object of type 'SetWifiCountryCode-response"
  "kortex_driver/SetWifiCountryCodeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetWifiCountryCode-response>)))
  "Returns md5sum for a message object of type '<SetWifiCountryCode-response>"
  "7960dab80443332660b3bb4bf774c9ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetWifiCountryCode-response)))
  "Returns md5sum for a message object of type 'SetWifiCountryCode-response"
  "7960dab80443332660b3bb4bf774c9ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetWifiCountryCode-response>)))
  "Returns full string definition for message of type '<SetWifiCountryCode-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetWifiCountryCode-response)))
  "Returns full string definition for message of type 'SetWifiCountryCode-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetWifiCountryCode-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetWifiCountryCode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetWifiCountryCode-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetWifiCountryCode)))
  'SetWifiCountryCode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetWifiCountryCode)))
  'SetWifiCountryCode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetWifiCountryCode)))
  "Returns string type for a service object of type '<SetWifiCountryCode>"
  "kortex_driver/SetWifiCountryCode")