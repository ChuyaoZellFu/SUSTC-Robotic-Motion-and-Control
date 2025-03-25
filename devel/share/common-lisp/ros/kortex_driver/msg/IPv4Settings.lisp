; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude IPv4Settings.msg.html

(cl:defclass <IPv4Settings> (roslisp-msg-protocol:ros-message)
  ((ipv4_address
    :reader ipv4_address
    :initarg :ipv4_address
    :type cl:integer
    :initform 0)
   (ipv4_subnet_mask
    :reader ipv4_subnet_mask
    :initarg :ipv4_subnet_mask
    :type cl:integer
    :initform 0)
   (ipv4_default_gateway
    :reader ipv4_default_gateway
    :initarg :ipv4_default_gateway
    :type cl:integer
    :initform 0))
)

(cl:defclass IPv4Settings (<IPv4Settings>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IPv4Settings>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IPv4Settings)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<IPv4Settings> is deprecated: use kortex_driver-msg:IPv4Settings instead.")))

(cl:ensure-generic-function 'ipv4_address-val :lambda-list '(m))
(cl:defmethod ipv4_address-val ((m <IPv4Settings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:ipv4_address-val is deprecated.  Use kortex_driver-msg:ipv4_address instead.")
  (ipv4_address m))

(cl:ensure-generic-function 'ipv4_subnet_mask-val :lambda-list '(m))
(cl:defmethod ipv4_subnet_mask-val ((m <IPv4Settings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:ipv4_subnet_mask-val is deprecated.  Use kortex_driver-msg:ipv4_subnet_mask instead.")
  (ipv4_subnet_mask m))

(cl:ensure-generic-function 'ipv4_default_gateway-val :lambda-list '(m))
(cl:defmethod ipv4_default_gateway-val ((m <IPv4Settings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:ipv4_default_gateway-val is deprecated.  Use kortex_driver-msg:ipv4_default_gateway instead.")
  (ipv4_default_gateway m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IPv4Settings>) ostream)
  "Serializes a message object of type '<IPv4Settings>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ipv4_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ipv4_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ipv4_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ipv4_address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ipv4_subnet_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ipv4_subnet_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ipv4_subnet_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ipv4_subnet_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ipv4_default_gateway)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ipv4_default_gateway)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ipv4_default_gateway)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ipv4_default_gateway)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IPv4Settings>) istream)
  "Deserializes a message object of type '<IPv4Settings>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ipv4_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ipv4_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ipv4_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ipv4_address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ipv4_subnet_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ipv4_subnet_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ipv4_subnet_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ipv4_subnet_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ipv4_default_gateway)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ipv4_default_gateway)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ipv4_default_gateway)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ipv4_default_gateway)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IPv4Settings>)))
  "Returns string type for a message object of type '<IPv4Settings>"
  "kortex_driver/IPv4Settings")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IPv4Settings)))
  "Returns string type for a message object of type 'IPv4Settings"
  "kortex_driver/IPv4Settings")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IPv4Settings>)))
  "Returns md5sum for a message object of type '<IPv4Settings>"
  "485e8d7e0822786c2180cac4b25f7434")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IPv4Settings)))
  "Returns md5sum for a message object of type 'IPv4Settings"
  "485e8d7e0822786c2180cac4b25f7434")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IPv4Settings>)))
  "Returns full string definition for message of type '<IPv4Settings>"
  (cl:format cl:nil "~%uint32 ipv4_address~%uint32 ipv4_subnet_mask~%uint32 ipv4_default_gateway~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IPv4Settings)))
  "Returns full string definition for message of type 'IPv4Settings"
  (cl:format cl:nil "~%uint32 ipv4_address~%uint32 ipv4_subnet_mask~%uint32 ipv4_default_gateway~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IPv4Settings>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IPv4Settings>))
  "Converts a ROS message object to a list"
  (cl:list 'IPv4Settings
    (cl:cons ':ipv4_address (ipv4_address msg))
    (cl:cons ':ipv4_subnet_mask (ipv4_subnet_mask msg))
    (cl:cons ':ipv4_default_gateway (ipv4_default_gateway msg))
))
