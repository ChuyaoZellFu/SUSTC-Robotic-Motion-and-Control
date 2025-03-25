; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude IPv4Configuration.msg.html

(cl:defclass <IPv4Configuration> (roslisp-msg-protocol:ros-message)
  ((ip_address
    :reader ip_address
    :initarg :ip_address
    :type cl:integer
    :initform 0)
   (subnet_mask
    :reader subnet_mask
    :initarg :subnet_mask
    :type cl:integer
    :initform 0)
   (default_gateway
    :reader default_gateway
    :initarg :default_gateway
    :type cl:integer
    :initform 0)
   (dhcp_enabled
    :reader dhcp_enabled
    :initarg :dhcp_enabled
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass IPv4Configuration (<IPv4Configuration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IPv4Configuration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IPv4Configuration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<IPv4Configuration> is deprecated: use kortex_driver-msg:IPv4Configuration instead.")))

(cl:ensure-generic-function 'ip_address-val :lambda-list '(m))
(cl:defmethod ip_address-val ((m <IPv4Configuration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:ip_address-val is deprecated.  Use kortex_driver-msg:ip_address instead.")
  (ip_address m))

(cl:ensure-generic-function 'subnet_mask-val :lambda-list '(m))
(cl:defmethod subnet_mask-val ((m <IPv4Configuration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:subnet_mask-val is deprecated.  Use kortex_driver-msg:subnet_mask instead.")
  (subnet_mask m))

(cl:ensure-generic-function 'default_gateway-val :lambda-list '(m))
(cl:defmethod default_gateway-val ((m <IPv4Configuration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:default_gateway-val is deprecated.  Use kortex_driver-msg:default_gateway instead.")
  (default_gateway m))

(cl:ensure-generic-function 'dhcp_enabled-val :lambda-list '(m))
(cl:defmethod dhcp_enabled-val ((m <IPv4Configuration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:dhcp_enabled-val is deprecated.  Use kortex_driver-msg:dhcp_enabled instead.")
  (dhcp_enabled m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IPv4Configuration>) ostream)
  "Serializes a message object of type '<IPv4Configuration>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ip_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ip_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ip_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ip_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'subnet_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'subnet_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'subnet_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'subnet_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'default_gateway)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'default_gateway)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'default_gateway)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'default_gateway)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dhcp_enabled) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IPv4Configuration>) istream)
  "Deserializes a message object of type '<IPv4Configuration>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ip_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ip_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ip_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ip_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'subnet_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'subnet_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'subnet_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'subnet_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'default_gateway)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'default_gateway)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'default_gateway)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'default_gateway)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dhcp_enabled) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IPv4Configuration>)))
  "Returns string type for a message object of type '<IPv4Configuration>"
  "kortex_driver/IPv4Configuration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IPv4Configuration)))
  "Returns string type for a message object of type 'IPv4Configuration"
  "kortex_driver/IPv4Configuration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IPv4Configuration>)))
  "Returns md5sum for a message object of type '<IPv4Configuration>"
  "938bacc92850e7d808a96d82d5b98e81")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IPv4Configuration)))
  "Returns md5sum for a message object of type 'IPv4Configuration"
  "938bacc92850e7d808a96d82d5b98e81")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IPv4Configuration>)))
  "Returns full string definition for message of type '<IPv4Configuration>"
  (cl:format cl:nil "~%uint32 ip_address~%uint32 subnet_mask~%uint32 default_gateway~%bool dhcp_enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IPv4Configuration)))
  "Returns full string definition for message of type 'IPv4Configuration"
  (cl:format cl:nil "~%uint32 ip_address~%uint32 subnet_mask~%uint32 default_gateway~%bool dhcp_enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IPv4Configuration>))
  (cl:+ 0
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IPv4Configuration>))
  "Converts a ROS message object to a list"
  (cl:list 'IPv4Configuration
    (cl:cons ':ip_address (ip_address msg))
    (cl:cons ':subnet_mask (subnet_mask msg))
    (cl:cons ':default_gateway (default_gateway msg))
    (cl:cons ':dhcp_enabled (dhcp_enabled msg))
))
