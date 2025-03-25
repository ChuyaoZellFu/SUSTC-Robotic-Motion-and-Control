; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CartesianLimitation.msg.html

(cl:defclass <CartesianLimitation> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (translation
    :reader translation
    :initarg :translation
    :type cl:float
    :initform 0.0)
   (orientation
    :reader orientation
    :initarg :orientation
    :type cl:float
    :initform 0.0))
)

(cl:defclass CartesianLimitation (<CartesianLimitation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CartesianLimitation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CartesianLimitation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CartesianLimitation> is deprecated: use kortex_driver-msg:CartesianLimitation instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <CartesianLimitation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:type-val is deprecated.  Use kortex_driver-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'translation-val :lambda-list '(m))
(cl:defmethod translation-val ((m <CartesianLimitation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:translation-val is deprecated.  Use kortex_driver-msg:translation instead.")
  (translation m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <CartesianLimitation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:orientation-val is deprecated.  Use kortex_driver-msg:orientation instead.")
  (orientation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CartesianLimitation>) ostream)
  "Serializes a message object of type '<CartesianLimitation>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'type)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'translation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'orientation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CartesianLimitation>) istream)
  "Deserializes a message object of type '<CartesianLimitation>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'translation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'orientation) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CartesianLimitation>)))
  "Returns string type for a message object of type '<CartesianLimitation>"
  "kortex_driver/CartesianLimitation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartesianLimitation)))
  "Returns string type for a message object of type 'CartesianLimitation"
  "kortex_driver/CartesianLimitation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CartesianLimitation>)))
  "Returns md5sum for a message object of type '<CartesianLimitation>"
  "2764a5779bb20a3b4f16f3cce1cf0f4c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CartesianLimitation)))
  "Returns md5sum for a message object of type 'CartesianLimitation"
  "2764a5779bb20a3b4f16f3cce1cf0f4c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CartesianLimitation>)))
  "Returns full string definition for message of type '<CartesianLimitation>"
  (cl:format cl:nil "~%uint32 type~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CartesianLimitation)))
  "Returns full string definition for message of type 'CartesianLimitation"
  (cl:format cl:nil "~%uint32 type~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CartesianLimitation>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CartesianLimitation>))
  "Converts a ROS message object to a list"
  (cl:list 'CartesianLimitation
    (cl:cons ':type (type msg))
    (cl:cons ':translation (translation msg))
    (cl:cons ':orientation (orientation msg))
))
