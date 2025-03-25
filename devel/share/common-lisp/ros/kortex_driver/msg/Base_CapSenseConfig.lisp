; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Base_CapSenseConfig.msg.html

(cl:defclass <Base_CapSenseConfig> (roslisp-msg-protocol:ros-message)
  ((identifier
    :reader identifier
    :initarg :identifier
    :type cl:integer
    :initform 0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0)
   (threshold_a
    :reader threshold_a
    :initarg :threshold_a
    :type cl:float
    :initform 0.0)
   (threshold_b
    :reader threshold_b
    :initarg :threshold_b
    :type cl:float
    :initform 0.0)
   (sensitivity_a
    :reader sensitivity_a
    :initarg :sensitivity_a
    :type cl:float
    :initform 0.0)
   (sensitivity_b
    :reader sensitivity_b
    :initarg :sensitivity_b
    :type cl:float
    :initform 0.0))
)

(cl:defclass Base_CapSenseConfig (<Base_CapSenseConfig>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Base_CapSenseConfig>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Base_CapSenseConfig)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Base_CapSenseConfig> is deprecated: use kortex_driver-msg:Base_CapSenseConfig instead.")))

(cl:ensure-generic-function 'identifier-val :lambda-list '(m))
(cl:defmethod identifier-val ((m <Base_CapSenseConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:identifier-val is deprecated.  Use kortex_driver-msg:identifier instead.")
  (identifier m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <Base_CapSenseConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:mode-val is deprecated.  Use kortex_driver-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'threshold_a-val :lambda-list '(m))
(cl:defmethod threshold_a-val ((m <Base_CapSenseConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:threshold_a-val is deprecated.  Use kortex_driver-msg:threshold_a instead.")
  (threshold_a m))

(cl:ensure-generic-function 'threshold_b-val :lambda-list '(m))
(cl:defmethod threshold_b-val ((m <Base_CapSenseConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:threshold_b-val is deprecated.  Use kortex_driver-msg:threshold_b instead.")
  (threshold_b m))

(cl:ensure-generic-function 'sensitivity_a-val :lambda-list '(m))
(cl:defmethod sensitivity_a-val ((m <Base_CapSenseConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:sensitivity_a-val is deprecated.  Use kortex_driver-msg:sensitivity_a instead.")
  (sensitivity_a m))

(cl:ensure-generic-function 'sensitivity_b-val :lambda-list '(m))
(cl:defmethod sensitivity_b-val ((m <Base_CapSenseConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:sensitivity_b-val is deprecated.  Use kortex_driver-msg:sensitivity_b instead.")
  (sensitivity_b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Base_CapSenseConfig>) ostream)
  "Serializes a message object of type '<Base_CapSenseConfig>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mode)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'threshold_a))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'threshold_b))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sensitivity_a))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sensitivity_b))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Base_CapSenseConfig>) istream)
  "Deserializes a message object of type '<Base_CapSenseConfig>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'threshold_a) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'threshold_b) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sensitivity_a) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sensitivity_b) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Base_CapSenseConfig>)))
  "Returns string type for a message object of type '<Base_CapSenseConfig>"
  "kortex_driver/Base_CapSenseConfig")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_CapSenseConfig)))
  "Returns string type for a message object of type 'Base_CapSenseConfig"
  "kortex_driver/Base_CapSenseConfig")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Base_CapSenseConfig>)))
  "Returns md5sum for a message object of type '<Base_CapSenseConfig>"
  "f62e507319ac72b37a316fe4248e2e5a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Base_CapSenseConfig)))
  "Returns md5sum for a message object of type 'Base_CapSenseConfig"
  "f62e507319ac72b37a316fe4248e2e5a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Base_CapSenseConfig>)))
  "Returns full string definition for message of type '<Base_CapSenseConfig>"
  (cl:format cl:nil "~%uint32 identifier~%uint32 mode~%float32 threshold_a~%float32 threshold_b~%float32 sensitivity_a~%float32 sensitivity_b~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Base_CapSenseConfig)))
  "Returns full string definition for message of type 'Base_CapSenseConfig"
  (cl:format cl:nil "~%uint32 identifier~%uint32 mode~%float32 threshold_a~%float32 threshold_b~%float32 sensitivity_a~%float32 sensitivity_b~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Base_CapSenseConfig>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Base_CapSenseConfig>))
  "Converts a ROS message object to a list"
  (cl:list 'Base_CapSenseConfig
    (cl:cons ':identifier (identifier msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':threshold_a (threshold_a msg))
    (cl:cons ':threshold_b (threshold_b msg))
    (cl:cons ':sensitivity_a (sensitivity_a msg))
    (cl:cons ':sensitivity_b (sensitivity_b msg))
))
