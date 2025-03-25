; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetIPv4Settings-request.msg.html

(cl:defclass <GetIPv4Settings-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetIPv4Settings-request (<GetIPv4Settings-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetIPv4Settings-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetIPv4Settings-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetIPv4Settings-request> is deprecated: use kortex_driver-srv:GetIPv4Settings-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetIPv4Settings-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetIPv4Settings-request>) ostream)
  "Serializes a message object of type '<GetIPv4Settings-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetIPv4Settings-request>) istream)
  "Deserializes a message object of type '<GetIPv4Settings-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetIPv4Settings-request>)))
  "Returns string type for a service object of type '<GetIPv4Settings-request>"
  "kortex_driver/GetIPv4SettingsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIPv4Settings-request)))
  "Returns string type for a service object of type 'GetIPv4Settings-request"
  "kortex_driver/GetIPv4SettingsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetIPv4Settings-request>)))
  "Returns md5sum for a message object of type '<GetIPv4Settings-request>"
  "e77074eb046f4a5b909d72ae5404567e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetIPv4Settings-request)))
  "Returns md5sum for a message object of type 'GetIPv4Settings-request"
  "e77074eb046f4a5b909d72ae5404567e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetIPv4Settings-request>)))
  "Returns full string definition for message of type '<GetIPv4Settings-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetIPv4Settings-request)))
  "Returns full string definition for message of type 'GetIPv4Settings-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetIPv4Settings-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetIPv4Settings-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetIPv4Settings-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetIPv4Settings-response.msg.html

(cl:defclass <GetIPv4Settings-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:IPv4Settings
    :initform (cl:make-instance 'kortex_driver-msg:IPv4Settings)))
)

(cl:defclass GetIPv4Settings-response (<GetIPv4Settings-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetIPv4Settings-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetIPv4Settings-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetIPv4Settings-response> is deprecated: use kortex_driver-srv:GetIPv4Settings-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetIPv4Settings-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetIPv4Settings-response>) ostream)
  "Serializes a message object of type '<GetIPv4Settings-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetIPv4Settings-response>) istream)
  "Deserializes a message object of type '<GetIPv4Settings-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetIPv4Settings-response>)))
  "Returns string type for a service object of type '<GetIPv4Settings-response>"
  "kortex_driver/GetIPv4SettingsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIPv4Settings-response)))
  "Returns string type for a service object of type 'GetIPv4Settings-response"
  "kortex_driver/GetIPv4SettingsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetIPv4Settings-response>)))
  "Returns md5sum for a message object of type '<GetIPv4Settings-response>"
  "e77074eb046f4a5b909d72ae5404567e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetIPv4Settings-response)))
  "Returns md5sum for a message object of type 'GetIPv4Settings-response"
  "e77074eb046f4a5b909d72ae5404567e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetIPv4Settings-response>)))
  "Returns full string definition for message of type '<GetIPv4Settings-response>"
  (cl:format cl:nil "IPv4Settings output~%~%================================================================================~%MSG: kortex_driver/IPv4Settings~%~%uint32 ipv4_address~%uint32 ipv4_subnet_mask~%uint32 ipv4_default_gateway~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetIPv4Settings-response)))
  "Returns full string definition for message of type 'GetIPv4Settings-response"
  (cl:format cl:nil "IPv4Settings output~%~%================================================================================~%MSG: kortex_driver/IPv4Settings~%~%uint32 ipv4_address~%uint32 ipv4_subnet_mask~%uint32 ipv4_default_gateway~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetIPv4Settings-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetIPv4Settings-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetIPv4Settings-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetIPv4Settings)))
  'GetIPv4Settings-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetIPv4Settings)))
  'GetIPv4Settings-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIPv4Settings)))
  "Returns string type for a service object of type '<GetIPv4Settings>"
  "kortex_driver/GetIPv4Settings")