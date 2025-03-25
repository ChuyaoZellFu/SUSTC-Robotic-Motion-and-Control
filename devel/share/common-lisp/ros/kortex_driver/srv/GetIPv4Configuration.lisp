; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetIPv4Configuration-request.msg.html

(cl:defclass <GetIPv4Configuration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:NetworkHandle
    :initform (cl:make-instance 'kortex_driver-msg:NetworkHandle)))
)

(cl:defclass GetIPv4Configuration-request (<GetIPv4Configuration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetIPv4Configuration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetIPv4Configuration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetIPv4Configuration-request> is deprecated: use kortex_driver-srv:GetIPv4Configuration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetIPv4Configuration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetIPv4Configuration-request>) ostream)
  "Serializes a message object of type '<GetIPv4Configuration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetIPv4Configuration-request>) istream)
  "Deserializes a message object of type '<GetIPv4Configuration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetIPv4Configuration-request>)))
  "Returns string type for a service object of type '<GetIPv4Configuration-request>"
  "kortex_driver/GetIPv4ConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIPv4Configuration-request)))
  "Returns string type for a service object of type 'GetIPv4Configuration-request"
  "kortex_driver/GetIPv4ConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetIPv4Configuration-request>)))
  "Returns md5sum for a message object of type '<GetIPv4Configuration-request>"
  "d1ab60af0d1991268832344a0f905c68")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetIPv4Configuration-request)))
  "Returns md5sum for a message object of type 'GetIPv4Configuration-request"
  "d1ab60af0d1991268832344a0f905c68")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetIPv4Configuration-request>)))
  "Returns full string definition for message of type '<GetIPv4Configuration-request>"
  (cl:format cl:nil "NetworkHandle input~%~%================================================================================~%MSG: kortex_driver/NetworkHandle~%~%uint32 type~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetIPv4Configuration-request)))
  "Returns full string definition for message of type 'GetIPv4Configuration-request"
  (cl:format cl:nil "NetworkHandle input~%~%================================================================================~%MSG: kortex_driver/NetworkHandle~%~%uint32 type~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetIPv4Configuration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetIPv4Configuration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetIPv4Configuration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetIPv4Configuration-response.msg.html

(cl:defclass <GetIPv4Configuration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:IPv4Configuration
    :initform (cl:make-instance 'kortex_driver-msg:IPv4Configuration)))
)

(cl:defclass GetIPv4Configuration-response (<GetIPv4Configuration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetIPv4Configuration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetIPv4Configuration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetIPv4Configuration-response> is deprecated: use kortex_driver-srv:GetIPv4Configuration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetIPv4Configuration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetIPv4Configuration-response>) ostream)
  "Serializes a message object of type '<GetIPv4Configuration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetIPv4Configuration-response>) istream)
  "Deserializes a message object of type '<GetIPv4Configuration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetIPv4Configuration-response>)))
  "Returns string type for a service object of type '<GetIPv4Configuration-response>"
  "kortex_driver/GetIPv4ConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIPv4Configuration-response)))
  "Returns string type for a service object of type 'GetIPv4Configuration-response"
  "kortex_driver/GetIPv4ConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetIPv4Configuration-response>)))
  "Returns md5sum for a message object of type '<GetIPv4Configuration-response>"
  "d1ab60af0d1991268832344a0f905c68")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetIPv4Configuration-response)))
  "Returns md5sum for a message object of type 'GetIPv4Configuration-response"
  "d1ab60af0d1991268832344a0f905c68")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetIPv4Configuration-response>)))
  "Returns full string definition for message of type '<GetIPv4Configuration-response>"
  (cl:format cl:nil "IPv4Configuration output~%~%================================================================================~%MSG: kortex_driver/IPv4Configuration~%~%uint32 ip_address~%uint32 subnet_mask~%uint32 default_gateway~%bool dhcp_enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetIPv4Configuration-response)))
  "Returns full string definition for message of type 'GetIPv4Configuration-response"
  (cl:format cl:nil "IPv4Configuration output~%~%================================================================================~%MSG: kortex_driver/IPv4Configuration~%~%uint32 ip_address~%uint32 subnet_mask~%uint32 default_gateway~%bool dhcp_enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetIPv4Configuration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetIPv4Configuration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetIPv4Configuration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetIPv4Configuration)))
  'GetIPv4Configuration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetIPv4Configuration)))
  'GetIPv4Configuration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIPv4Configuration)))
  "Returns string type for a service object of type '<GetIPv4Configuration>"
  "kortex_driver/GetIPv4Configuration")