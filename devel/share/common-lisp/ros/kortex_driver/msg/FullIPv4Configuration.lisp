; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude FullIPv4Configuration.msg.html

(cl:defclass <FullIPv4Configuration> (roslisp-msg-protocol:ros-message)
  ((handle
    :reader handle
    :initarg :handle
    :type kortex_driver-msg:NetworkHandle
    :initform (cl:make-instance 'kortex_driver-msg:NetworkHandle))
   (ipv4_configuration
    :reader ipv4_configuration
    :initarg :ipv4_configuration
    :type kortex_driver-msg:IPv4Configuration
    :initform (cl:make-instance 'kortex_driver-msg:IPv4Configuration)))
)

(cl:defclass FullIPv4Configuration (<FullIPv4Configuration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FullIPv4Configuration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FullIPv4Configuration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<FullIPv4Configuration> is deprecated: use kortex_driver-msg:FullIPv4Configuration instead.")))

(cl:ensure-generic-function 'handle-val :lambda-list '(m))
(cl:defmethod handle-val ((m <FullIPv4Configuration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:handle-val is deprecated.  Use kortex_driver-msg:handle instead.")
  (handle m))

(cl:ensure-generic-function 'ipv4_configuration-val :lambda-list '(m))
(cl:defmethod ipv4_configuration-val ((m <FullIPv4Configuration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:ipv4_configuration-val is deprecated.  Use kortex_driver-msg:ipv4_configuration instead.")
  (ipv4_configuration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FullIPv4Configuration>) ostream)
  "Serializes a message object of type '<FullIPv4Configuration>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'handle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ipv4_configuration) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FullIPv4Configuration>) istream)
  "Deserializes a message object of type '<FullIPv4Configuration>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'handle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ipv4_configuration) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FullIPv4Configuration>)))
  "Returns string type for a message object of type '<FullIPv4Configuration>"
  "kortex_driver/FullIPv4Configuration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FullIPv4Configuration)))
  "Returns string type for a message object of type 'FullIPv4Configuration"
  "kortex_driver/FullIPv4Configuration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FullIPv4Configuration>)))
  "Returns md5sum for a message object of type '<FullIPv4Configuration>"
  "b0ba6974979f83d97757b00aa8a1bc7d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FullIPv4Configuration)))
  "Returns md5sum for a message object of type 'FullIPv4Configuration"
  "b0ba6974979f83d97757b00aa8a1bc7d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FullIPv4Configuration>)))
  "Returns full string definition for message of type '<FullIPv4Configuration>"
  (cl:format cl:nil "~%NetworkHandle handle~%IPv4Configuration ipv4_configuration~%================================================================================~%MSG: kortex_driver/NetworkHandle~%~%uint32 type~%================================================================================~%MSG: kortex_driver/IPv4Configuration~%~%uint32 ip_address~%uint32 subnet_mask~%uint32 default_gateway~%bool dhcp_enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FullIPv4Configuration)))
  "Returns full string definition for message of type 'FullIPv4Configuration"
  (cl:format cl:nil "~%NetworkHandle handle~%IPv4Configuration ipv4_configuration~%================================================================================~%MSG: kortex_driver/NetworkHandle~%~%uint32 type~%================================================================================~%MSG: kortex_driver/IPv4Configuration~%~%uint32 ip_address~%uint32 subnet_mask~%uint32 default_gateway~%bool dhcp_enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FullIPv4Configuration>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'handle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ipv4_configuration))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FullIPv4Configuration>))
  "Converts a ROS message object to a list"
  (cl:list 'FullIPv4Configuration
    (cl:cons ':handle (handle msg))
    (cl:cons ':ipv4_configuration (ipv4_configuration msg))
))
