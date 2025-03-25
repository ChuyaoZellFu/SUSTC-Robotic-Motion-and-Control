; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetIPv4Configuration-request.msg.html

(cl:defclass <SetIPv4Configuration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:FullIPv4Configuration
    :initform (cl:make-instance 'kortex_driver-msg:FullIPv4Configuration)))
)

(cl:defclass SetIPv4Configuration-request (<SetIPv4Configuration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetIPv4Configuration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetIPv4Configuration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetIPv4Configuration-request> is deprecated: use kortex_driver-srv:SetIPv4Configuration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetIPv4Configuration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetIPv4Configuration-request>) ostream)
  "Serializes a message object of type '<SetIPv4Configuration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetIPv4Configuration-request>) istream)
  "Deserializes a message object of type '<SetIPv4Configuration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetIPv4Configuration-request>)))
  "Returns string type for a service object of type '<SetIPv4Configuration-request>"
  "kortex_driver/SetIPv4ConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetIPv4Configuration-request)))
  "Returns string type for a service object of type 'SetIPv4Configuration-request"
  "kortex_driver/SetIPv4ConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetIPv4Configuration-request>)))
  "Returns md5sum for a message object of type '<SetIPv4Configuration-request>"
  "efb45709f10aca05c41eab84d8334547")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetIPv4Configuration-request)))
  "Returns md5sum for a message object of type 'SetIPv4Configuration-request"
  "efb45709f10aca05c41eab84d8334547")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetIPv4Configuration-request>)))
  "Returns full string definition for message of type '<SetIPv4Configuration-request>"
  (cl:format cl:nil "FullIPv4Configuration input~%~%================================================================================~%MSG: kortex_driver/FullIPv4Configuration~%~%NetworkHandle handle~%IPv4Configuration ipv4_configuration~%================================================================================~%MSG: kortex_driver/NetworkHandle~%~%uint32 type~%================================================================================~%MSG: kortex_driver/IPv4Configuration~%~%uint32 ip_address~%uint32 subnet_mask~%uint32 default_gateway~%bool dhcp_enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetIPv4Configuration-request)))
  "Returns full string definition for message of type 'SetIPv4Configuration-request"
  (cl:format cl:nil "FullIPv4Configuration input~%~%================================================================================~%MSG: kortex_driver/FullIPv4Configuration~%~%NetworkHandle handle~%IPv4Configuration ipv4_configuration~%================================================================================~%MSG: kortex_driver/NetworkHandle~%~%uint32 type~%================================================================================~%MSG: kortex_driver/IPv4Configuration~%~%uint32 ip_address~%uint32 subnet_mask~%uint32 default_gateway~%bool dhcp_enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetIPv4Configuration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetIPv4Configuration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetIPv4Configuration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetIPv4Configuration-response.msg.html

(cl:defclass <SetIPv4Configuration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetIPv4Configuration-response (<SetIPv4Configuration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetIPv4Configuration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetIPv4Configuration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetIPv4Configuration-response> is deprecated: use kortex_driver-srv:SetIPv4Configuration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetIPv4Configuration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetIPv4Configuration-response>) ostream)
  "Serializes a message object of type '<SetIPv4Configuration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetIPv4Configuration-response>) istream)
  "Deserializes a message object of type '<SetIPv4Configuration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetIPv4Configuration-response>)))
  "Returns string type for a service object of type '<SetIPv4Configuration-response>"
  "kortex_driver/SetIPv4ConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetIPv4Configuration-response)))
  "Returns string type for a service object of type 'SetIPv4Configuration-response"
  "kortex_driver/SetIPv4ConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetIPv4Configuration-response>)))
  "Returns md5sum for a message object of type '<SetIPv4Configuration-response>"
  "efb45709f10aca05c41eab84d8334547")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetIPv4Configuration-response)))
  "Returns md5sum for a message object of type 'SetIPv4Configuration-response"
  "efb45709f10aca05c41eab84d8334547")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetIPv4Configuration-response>)))
  "Returns full string definition for message of type '<SetIPv4Configuration-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetIPv4Configuration-response)))
  "Returns full string definition for message of type 'SetIPv4Configuration-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetIPv4Configuration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetIPv4Configuration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetIPv4Configuration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetIPv4Configuration)))
  'SetIPv4Configuration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetIPv4Configuration)))
  'SetIPv4Configuration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetIPv4Configuration)))
  "Returns string type for a service object of type '<SetIPv4Configuration>"
  "kortex_driver/SetIPv4Configuration")