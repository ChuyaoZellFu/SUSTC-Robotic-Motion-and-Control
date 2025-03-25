; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude IPv4Information.msg.html

(cl:defclass <IPv4Information> (roslisp-msg-protocol:ros-message)
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
    :initform 0))
)

(cl:defclass IPv4Information (<IPv4Information>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IPv4Information>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IPv4Information)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<IPv4Information> is deprecated: use kortex_driver-msg:IPv4Information instead.")))

(cl:ensure-generic-function 'ip_address-val :lambda-list '(m))
(cl:defmethod ip_address-val ((m <IPv4Information>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:ip_address-val is deprecated.  Use kortex_driver-msg:ip_address instead.")
  (ip_address m))

(cl:ensure-generic-function 'subnet_mask-val :lambda-list '(m))
(cl:defmethod subnet_mask-val ((m <IPv4Information>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:subnet_mask-val is deprecated.  Use kortex_driver-msg:subnet_mask instead.")
  (subnet_mask m))

(cl:ensure-generic-function 'default_gateway-val :lambda-list '(m))
(cl:defmethod default_gateway-val ((m <IPv4Information>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:default_gateway-val is deprecated.  Use kortex_driver-msg:default_gateway instead.")
  (default_gateway m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IPv4Information>) ostream)
  "Serializes a message object of type '<IPv4Information>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IPv4Information>) istream)
  "Deserializes a message object of type '<IPv4Information>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IPv4Information>)))
  "Returns string type for a message object of type '<IPv4Information>"
  "kortex_driver/IPv4Information")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IPv4Information)))
  "Returns string type for a message object of type 'IPv4Information"
  "kortex_driver/IPv4Information")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IPv4Information>)))
  "Returns md5sum for a message object of type '<IPv4Information>"
  "8070c0574c2b6ebcf5c917ce0b71133c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IPv4Information)))
  "Returns md5sum for a message object of type 'IPv4Information"
  "8070c0574c2b6ebcf5c917ce0b71133c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IPv4Information>)))
  "Returns full string definition for message of type '<IPv4Information>"
  (cl:format cl:nil "~%uint32 ip_address~%uint32 subnet_mask~%uint32 default_gateway~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IPv4Information)))
  "Returns full string definition for message of type 'IPv4Information"
  (cl:format cl:nil "~%uint32 ip_address~%uint32 subnet_mask~%uint32 default_gateway~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IPv4Information>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IPv4Information>))
  "Converts a ROS message object to a list"
  (cl:list 'IPv4Information
    (cl:cons ':ip_address (ip_address msg))
    (cl:cons ':subnet_mask (subnet_mask msg))
    (cl:cons ':default_gateway (default_gateway msg))
))
