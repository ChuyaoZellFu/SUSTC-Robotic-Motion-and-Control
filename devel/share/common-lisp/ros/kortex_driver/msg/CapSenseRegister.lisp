; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CapSenseRegister.msg.html

(cl:defclass <CapSenseRegister> (roslisp-msg-protocol:ros-message)
  ((address
    :reader address
    :initarg :address
    :type cl:integer
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0))
)

(cl:defclass CapSenseRegister (<CapSenseRegister>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CapSenseRegister>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CapSenseRegister)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CapSenseRegister> is deprecated: use kortex_driver-msg:CapSenseRegister instead.")))

(cl:ensure-generic-function 'address-val :lambda-list '(m))
(cl:defmethod address-val ((m <CapSenseRegister>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:address-val is deprecated.  Use kortex_driver-msg:address instead.")
  (address m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <CapSenseRegister>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:value-val is deprecated.  Use kortex_driver-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CapSenseRegister>) ostream)
  "Serializes a message object of type '<CapSenseRegister>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'address)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CapSenseRegister>) istream)
  "Deserializes a message object of type '<CapSenseRegister>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'address)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'value)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CapSenseRegister>)))
  "Returns string type for a message object of type '<CapSenseRegister>"
  "kortex_driver/CapSenseRegister")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CapSenseRegister)))
  "Returns string type for a message object of type 'CapSenseRegister"
  "kortex_driver/CapSenseRegister")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CapSenseRegister>)))
  "Returns md5sum for a message object of type '<CapSenseRegister>"
  "e812c91d734d8142f7286bd1229d1f0d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CapSenseRegister)))
  "Returns md5sum for a message object of type 'CapSenseRegister"
  "e812c91d734d8142f7286bd1229d1f0d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CapSenseRegister>)))
  "Returns full string definition for message of type '<CapSenseRegister>"
  (cl:format cl:nil "~%uint32 address~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CapSenseRegister)))
  "Returns full string definition for message of type 'CapSenseRegister"
  (cl:format cl:nil "~%uint32 address~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CapSenseRegister>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CapSenseRegister>))
  "Converts a ROS message object to a list"
  (cl:list 'CapSenseRegister
    (cl:cons ':address (address msg))
    (cl:cons ':value (value msg))
))
