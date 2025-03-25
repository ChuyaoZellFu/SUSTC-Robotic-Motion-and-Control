; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude VectorDriveParameters.msg.html

(cl:defclass <VectorDriveParameters> (roslisp-msg-protocol:ros-message)
  ((kpq
    :reader kpq
    :initarg :kpq
    :type cl:float
    :initform 0.0)
   (kiq
    :reader kiq
    :initarg :kiq
    :type cl:float
    :initform 0.0)
   (kpd
    :reader kpd
    :initarg :kpd
    :type cl:float
    :initform 0.0)
   (kid
    :reader kid
    :initarg :kid
    :type cl:float
    :initform 0.0))
)

(cl:defclass VectorDriveParameters (<VectorDriveParameters>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VectorDriveParameters>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VectorDriveParameters)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<VectorDriveParameters> is deprecated: use kortex_driver-msg:VectorDriveParameters instead.")))

(cl:ensure-generic-function 'kpq-val :lambda-list '(m))
(cl:defmethod kpq-val ((m <VectorDriveParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:kpq-val is deprecated.  Use kortex_driver-msg:kpq instead.")
  (kpq m))

(cl:ensure-generic-function 'kiq-val :lambda-list '(m))
(cl:defmethod kiq-val ((m <VectorDriveParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:kiq-val is deprecated.  Use kortex_driver-msg:kiq instead.")
  (kiq m))

(cl:ensure-generic-function 'kpd-val :lambda-list '(m))
(cl:defmethod kpd-val ((m <VectorDriveParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:kpd-val is deprecated.  Use kortex_driver-msg:kpd instead.")
  (kpd m))

(cl:ensure-generic-function 'kid-val :lambda-list '(m))
(cl:defmethod kid-val ((m <VectorDriveParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:kid-val is deprecated.  Use kortex_driver-msg:kid instead.")
  (kid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VectorDriveParameters>) ostream)
  "Serializes a message object of type '<VectorDriveParameters>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'kpq))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'kiq))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'kpd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'kid))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VectorDriveParameters>) istream)
  "Deserializes a message object of type '<VectorDriveParameters>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kpq) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kiq) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kpd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kid) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VectorDriveParameters>)))
  "Returns string type for a message object of type '<VectorDriveParameters>"
  "kortex_driver/VectorDriveParameters")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VectorDriveParameters)))
  "Returns string type for a message object of type 'VectorDriveParameters"
  "kortex_driver/VectorDriveParameters")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VectorDriveParameters>)))
  "Returns md5sum for a message object of type '<VectorDriveParameters>"
  "30e295016ac63da10cbe59e969bb6401")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VectorDriveParameters)))
  "Returns md5sum for a message object of type 'VectorDriveParameters"
  "30e295016ac63da10cbe59e969bb6401")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VectorDriveParameters>)))
  "Returns full string definition for message of type '<VectorDriveParameters>"
  (cl:format cl:nil "~%float32 kpq~%float32 kiq~%float32 kpd~%float32 kid~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VectorDriveParameters)))
  "Returns full string definition for message of type 'VectorDriveParameters"
  (cl:format cl:nil "~%float32 kpq~%float32 kiq~%float32 kpd~%float32 kid~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VectorDriveParameters>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VectorDriveParameters>))
  "Converts a ROS message object to a list"
  (cl:list 'VectorDriveParameters
    (cl:cons ':kpq (kpq msg))
    (cl:cons ':kiq (kiq msg))
    (cl:cons ':kpd (kpd msg))
    (cl:cons ':kid (kid msg))
))
