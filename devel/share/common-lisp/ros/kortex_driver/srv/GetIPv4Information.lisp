; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetIPv4Information-request.msg.html

(cl:defclass <GetIPv4Information-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NetworkHandle
    :initform (cl:make-instance 'kortex_driver-msg:NetworkHandle)))
)

(cl:defclass GetIPv4Information-request (<GetIPv4Information-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetIPv4Information-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetIPv4Information-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetIPv4Information-request> is deprecated: use kortex_driver-srv:GetIPv4Information-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetIPv4Information-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetIPv4Information-request>) ostream)
  "Serializes a message object of type '<GetIPv4Information-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetIPv4Information-request>) istream)
  "Deserializes a message object of type '<GetIPv4Information-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetIPv4Information-request>)))
  "Returns string type for a service object of type '<GetIPv4Information-request>"
  "kortex_driver/GetIPv4InformationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIPv4Information-request)))
  "Returns string type for a service object of type 'GetIPv4Information-request"
  "kortex_driver/GetIPv4InformationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetIPv4Information-request>)))
  "Returns md5sum for a message object of type '<GetIPv4Information-request>"
  "cf6aba6a62d331d4b104d99fe292f36c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetIPv4Information-request)))
  "Returns md5sum for a message object of type 'GetIPv4Information-request"
  "cf6aba6a62d331d4b104d99fe292f36c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetIPv4Information-request>)))
  "Returns full string definition for message of type '<GetIPv4Information-request>"
  (cl:format cl:nil "NetworkHandle input~%~%================================================================================~%MSG: kortex_driver/NetworkHandle~%~%uint32 type~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetIPv4Information-request)))
  "Returns full string definition for message of type 'GetIPv4Information-request"
  (cl:format cl:nil "NetworkHandle input~%~%================================================================================~%MSG: kortex_driver/NetworkHandle~%~%uint32 type~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetIPv4Information-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetIPv4Information-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetIPv4Information-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetIPv4Information-response.msg.html

(cl:defclass <GetIPv4Information-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:IPv4Information
    :initform (cl:make-instance 'kortex_driver-msg:IPv4Information)))
)

(cl:defclass GetIPv4Information-response (<GetIPv4Information-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetIPv4Information-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetIPv4Information-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetIPv4Information-response> is deprecated: use kortex_driver-srv:GetIPv4Information-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetIPv4Information-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetIPv4Information-response>) ostream)
  "Serializes a message object of type '<GetIPv4Information-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetIPv4Information-response>) istream)
  "Deserializes a message object of type '<GetIPv4Information-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetIPv4Information-response>)))
  "Returns string type for a service object of type '<GetIPv4Information-response>"
  "kortex_driver/GetIPv4InformationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIPv4Information-response)))
  "Returns string type for a service object of type 'GetIPv4Information-response"
  "kortex_driver/GetIPv4InformationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetIPv4Information-response>)))
  "Returns md5sum for a message object of type '<GetIPv4Information-response>"
  "cf6aba6a62d331d4b104d99fe292f36c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetIPv4Information-response)))
  "Returns md5sum for a message object of type 'GetIPv4Information-response"
  "cf6aba6a62d331d4b104d99fe292f36c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetIPv4Information-response>)))
  "Returns full string definition for message of type '<GetIPv4Information-response>"
  (cl:format cl:nil "IPv4Information output~%~%================================================================================~%MSG: kortex_driver/IPv4Information~%~%uint32 ip_address~%uint32 subnet_mask~%uint32 default_gateway~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetIPv4Information-response)))
  "Returns full string definition for message of type 'GetIPv4Information-response"
  (cl:format cl:nil "IPv4Information output~%~%================================================================================~%MSG: kortex_driver/IPv4Information~%~%uint32 ip_address~%uint32 subnet_mask~%uint32 default_gateway~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetIPv4Information-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetIPv4Information-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetIPv4Information-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetIPv4Information)))
  'GetIPv4Information-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetIPv4Information)))
  'GetIPv4Information-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIPv4Information)))
  "Returns string type for a service object of type '<GetIPv4Information>"
  "kortex_driver/GetIPv4Information")